use core::convert::TryFrom;
use core::fmt::{Display, Write};
use core::num::ParseIntError;
use core::str::{FromStr, Utf8Error};

use crate::ism43362::Ism43362;

use arrayvec::{ArrayString, ArrayVec, CapacityError};
use bstr::ByteSlice;
use defmt::Debug2Format;
use embassy::time::{Duration, Timer};
use embassy_stm32::peripherals::{PG11, PG12, PH1, SPI3};
use embassy_stm32::spi::Error as SpiError;
use httparse::{Request, Status};

const WIFI_NETWORKS: [(&[u8], &[u8]); 2] = [];

#[derive(defmt::Format)]
pub enum NetworkError {
    Spi(SpiError),
    Limit,
    Prompt,
    Parse(ParseError),
    CommandError,
    Json,
    Fmt,
    Disconnected,
}

#[derive(defmt::Format)]
pub enum ParseError {
    ResponsePrefix,
    ResponseSuffix,
    Delimiter,
    Utf8,
    Integer,
}

impl From<SpiError> for NetworkError {
    fn from(err: SpiError) -> Self {
        Self::Spi(err)
    }
}

impl From<CapacityError> for NetworkError {
    fn from(_: CapacityError) -> Self {
        Self::Limit
    }
}

impl From<Utf8Error> for NetworkError {
    fn from(_: Utf8Error) -> Self {
        Self::Parse(ParseError::Utf8)
    }
}

impl From<ParseIntError> for NetworkError {
    fn from(_: ParseIntError) -> Self {
        Self::Parse(ParseError::Integer)
    }
}

impl From<CommandError<'_>> for NetworkError {
    fn from(err: CommandError) -> Self {
        defmt::error!(
            "command failed: {} {}",
            Debug2Format(err.usage.as_bstr()),
            Debug2Format(err.data.as_bstr())
        );
        Self::CommandError
    }
}

impl From<serde_json_core::ser::Error> for NetworkError {
    fn from(err: serde_json_core::ser::Error) -> Self {
        defmt::error!("JSON serialization failed: {}", Debug2Format(&err));

        Self::Json
    }
}

impl From<core::fmt::Error> for NetworkError {
    fn from(_: core::fmt::Error) -> Self {
        Self::Fmt
    }
}

fn connect_set_ssid(ssid: &[u8]) -> Result<ArrayVec<u8, { 3 + 32 }>, CapacityError> {
    let mut command = ArrayVec::new();
    command.try_extend_from_slice(b"C1=")?;
    command.try_extend_from_slice(ssid)?;
    Ok(command)
}

fn connect_set_passphrase(passphrase: &[u8]) -> Result<ArrayVec<u8, { 3 + 63 }>, CapacityError> {
    let mut command = ArrayVec::new();
    command.try_extend_from_slice(b"C2=")?;
    command.try_extend_from_slice(passphrase)?;
    Ok(command)
}

#[embassy::task]
pub async fn network_task(mut wifi: Ism43362<'static, SPI3, PH1, PG12, PG11>) {
    loop {
        wifi.reset().await;

        if let Err(err) = network_task_inner(&mut wifi).await {
            defmt::error!("Network task failed: {}", err);
        }
        crate::with_network_info(|info| *info = None);

        Timer::after(Duration::from_secs(5)).await;
    }
}

async fn network_task_inner(
    wifi: &mut Ism43362<'static, SPI3, PH1, PG12, PG11>,
) -> Result<(), NetworkError> {
    let prompt = wifi.read().await?;
    if prompt.as_slice() != b"\r\n> " {
        defmt::error!("invalid prompt: {}", prompt.as_slice());
        return Err(NetworkError::Prompt);
    }

    let version = wifi.send_command(b"I?").await?;
    let version = parse_response(&version)??;
    defmt::info!(
        "WiFi module version information: {}",
        Debug2Format(version.as_bstr())
    );

    let res = wifi.send_command(b"CD").await?;
    let res = parse_response(&res)??;
    defmt::debug!("CD response: {}", Debug2Format(res.as_bstr()));

    let (ssid, ip) = 'connect: loop {
        let networks = wifi.send_command(b"F0").await?;
        let networks = parse_response(&networks)??;
        // We could parse the networks but there are two critical issues:
        //  * while the SSIDs are between quotes they are not escaped
        //  * a SSID can contain 1-32 of arbitrary octets
        // So let's cheat instead.
        for (ssid, password) in WIFI_NETWORKS {
            if networks.find(ssid).is_some() {
                let res = wifi.send_command(&connect_set_ssid(ssid)?).await?;
                let res = parse_response(&res)??;
                defmt::debug!("C1 response: {}", Debug2Format(res.as_bstr()));

                let res = wifi
                    .send_command(&connect_set_passphrase(password)?)
                    .await?;
                let res = parse_response(&res)??;
                defmt::debug!("C2 response: {}", Debug2Format(res.as_bstr()));

                // Set security type to 4=WPA2 Mixed
                let res = wifi.send_command(b"C3=4").await?;
                let res = parse_response(&res)??;
                defmt::debug!("C3 response: {}", Debug2Format(res.as_bstr()));

                // Enable DHCP
                let res = wifi.send_command(b"C4=1").await?;
                let res = parse_response(&res)??;
                defmt::debug!("C4 response: {}", Debug2Format(res.as_bstr()));

                // Connect
                let res = wifi.send_command(b"C0").await?;
                let res = parse_response(&res)??;
                defmt::debug!("C0 response: {}", Debug2Format(res.as_bstr()));

                break 'connect parse_join_message(res)?;
            }
        }
    };

    crate::with_network_info(|info| {
        let mut ssid_as_str = ArrayString::new();
        for chunk in ssid.utf8_chunks() {
            ssid_as_str.push_str(chunk.valid());
            if !chunk.invalid().is_empty() {
                ssid_as_str.push('\u{FFFD}');
            }
        }
        *info = Some((ssid_as_str, ip));
    });

    defmt::info!(
        "Connected to a WiFi network: SSID: {}; IP: {}",
        Debug2Format(ssid.as_slice().as_bstr()),
        ip
    );

    // Set socket
    let res = wifi.send_command(b"P0=0").await?;
    let res = parse_response(&res)??;
    defmt::info!("P0 response: {}", Debug2Format(res.as_bstr()));
    // Select TCP
    let res = wifi.send_command(b"P1=0").await?;
    let res = parse_response(&res)??;
    defmt::info!("P1 response: {}", Debug2Format(res.as_bstr()));
    // Set port to 80
    let res = wifi.send_command(b"P2=80").await?;
    let res = parse_response(&res)??;
    defmt::info!("P2 response: {}", Debug2Format(res.as_bstr()));
    // Set accept backlog to 8
    let res = wifi.send_command(b"P8=8").await?;
    let res = parse_response(&res)??;
    defmt::info!("P8 response: {}", Debug2Format(res.as_bstr()));
    // Start a TCP server in multi-accept mode
    let res = wifi.send_command(b"P5=11").await?;
    let res = parse_response(&res)??;
    defmt::info!("P5 response: {}", Debug2Format(res.as_bstr()));

    loop {
        let res = wifi.send_command(b"MR").await?;
        let res = parse_response(&res)??;
        let res = parse_async_message(res)?;

        if !res.is_empty() {
            defmt::info!("MR response: {}", Debug2Format(res.as_bstr()));
        }

        if res.starts_with(b"[TCP SVR] Accepted ") {
            if let Err(err) = handle_connection(wifi).await {
                defmt::error!("Connection handler failed: {}", err);
            }

            // Close connection and wait for the next one
            let res = wifi.send_command(b"P5=10").await?;
            let res = parse_response(&res)??;
            defmt::info!("P5 response: {}", Debug2Format(res.as_bstr()));
        } else {
            let res = wifi.send_command(b"CS").await?;
            let res = parse_response(&res)??;
            if res == b"0" {
                return Err(NetworkError::Disconnected);
            }

            Timer::after(Duration::from_millis(500)).await;
        }
    }
}

enum Route {
    GetIndex,
    GetMeasurements,
    GetMetrics,
}

impl Route {
    fn from_request(request: &Request) -> Status<Option<Self>> {
        match (request.method, request.path) {
            (Some("GET"), Some("/")) => Status::Complete(Some(Self::GetIndex)),
            (Some("GET"), Some("/measurements")) => Status::Complete(Some(Self::GetMeasurements)),
            (Some("GET"), Some("/metrics")) => Status::Complete(Some(Self::GetMetrics)),
            (Some(_), Some(_)) => Status::Complete(None),
            _ => Status::Partial,
        }
    }
}

const ERROR_400_BAD_REQUEST: &[u8] =
    b"HTTP/1.0 400 Bad Request\r\nContent-Type: text/plain\r\n\r\nHTTP/1.0 400 Bad Request";
const ERROR_404_NOT_FOUND: &[u8] =
    b"HTTP/1.0 404 Not Found\r\nContent-Type: text/plain\r\n\r\nHTTP/1.0 404 Not Found";
const ERROR_413_REQUEST_HEADER_FIELDS_TOO_LARGE: &[u8] = b"HTTP/1.0 431 Request Header Fields Too Large\r\nContent-Type: text/plain\r\n\r\nHTTP/1.0 431 Request Header Fields Too Large";

fn parse_http_request(request_buffer: &[u8]) -> Result<Status<Route>, &'static [u8]> {
    let mut request_headers = [httparse::EMPTY_HEADER; 16];
    let mut request = Request::new(&mut request_headers);
    let res = request.parse(&request_buffer);
    let route = Route::from_request(&request);

    match (res, route) {
        (_, Status::Complete(None)) => Err(ERROR_404_NOT_FOUND),
        (Ok(Status::Complete(_)), Status::Complete(Some(route))) => Ok(Status::Complete(route)),
        (Ok(Status::Complete(_)), Status::Partial) => Err(ERROR_404_NOT_FOUND),
        (Ok(Status::Partial), _) => Ok(Status::Partial),
        (Err(httparse::Error::TooManyHeaders), _) => Err(ERROR_413_REQUEST_HEADER_FIELDS_TOO_LARGE),
        (Err(err), _) => {
            defmt::error!("error parsing HTTP request: {}", Debug2Format(&err));
            Err(ERROR_400_BAD_REQUEST)
        }
    }
}

struct WriteAdaptor<'a, const CAP: usize>(&'a mut ArrayVec<u8, CAP>);

impl<const CAP: usize> Write for WriteAdaptor<'_, CAP> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.0
            .try_extend_from_slice(s.as_bytes())
            .map_err(|_| core::fmt::Error)
    }
}

async fn handle_connection(
    wifi: &mut Ism43362<'static, SPI3, PH1, PG12, PG11>,
) -> Result<(), NetworkError> {
    let mut request_buffer = ArrayVec::<u8, 2048>::new();

    let route = loop {
        let res = wifi.send_command(b"R0").await?;
        let res = parse_response(&res)??;

        if let Err(_) = request_buffer.try_extend_from_slice(res) {
            let res = wifi
                .send_s3_command(ERROR_413_REQUEST_HEADER_FIELDS_TOO_LARGE)
                .await?;
            let res = parse_response(&res)??;
            defmt::info!("S3 response: {}", Debug2Format(res.as_bstr()));

            return Ok(());
        }

        match parse_http_request(&request_buffer) {
            Ok(Status::Complete(route)) => break route,
            Ok(Status::Partial) => continue,
            Err(response) => {
                let res = wifi.send_s3_command(response).await?;
                let res = parse_response(&res)??;
                defmt::info!("S3 response: {}", Debug2Format(res.as_bstr()));
                return Ok(());
            }
        }
    };

    let mut body = ArrayVec::<u8, 2048>::new();
    let (headers, body) = match route {
        Route::GetIndex => (
            &b"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"[..],
            &include_bytes!("index.html")[..],
        ),
        Route::GetMeasurements => {
            while let Ok(()) = body.try_push(0) {}

            let readouts = crate::with_readouts(|readouts| *readouts);
            let bytes = serde_json_core::to_slice(&readouts, &mut body)?;
            body.truncate(bytes);

            (
                &b"HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n\r\n"[..],
                body.as_slice(),
            )
        }
        Route::GetMetrics => {
            let readouts = crate::with_readouts(|readouts| *readouts);

            {
                let mut adaptor = WriteAdaptor(&mut body);

                if let Some(measurement) = readouts.scd30 {
                    writeln!(adaptor, "scd30_co2 {}", measurement.co2)?;
                    writeln!(adaptor, "scd30_temperature {}", measurement.temperature)?;
                    writeln!(adaptor, "scd30_relative_humidity {}", measurement.humidity)?;
                }

                if let Some(measurement) = readouts.hp206c {
                    writeln!(adaptor, "hp206c_pressure {}", measurement.pressure)?;
                    writeln!(adaptor, "hp206c_temperature {}", measurement.temperature)?;
                }

                if let Some(measurement) = readouts.mix8410 {
                    writeln!(adaptor, "mix8410_o2_voltage {}", measurement.voltage).unwrap();
                    writeln!(
                        adaptor,
                        "mix8410_o2_concentration {}",
                        measurement.concentration
                    )
                    .unwrap();
                }
            }

            (
                &b"HTTP/1.0 200 OK\r\nContent-Type: text/plain; version=0.0.4\r\n\r\n"[..],
                body.as_slice(),
            )
        }
    };

    let res = wifi.send_s3_command(headers).await?;
    let res = parse_response(&res)??;
    defmt::info!("S3 response [headers]: {}", Debug2Format(res.as_bstr()));

    for chunk in body.chunks(1024) {
        let res = wifi.send_s3_command(chunk).await?;
        let res = parse_response(&res)??;
        defmt::info!("S3 response [body]: {}", Debug2Format(res.as_bstr()));
    }

    Ok(())
}

#[derive(defmt::Format)]
struct CommandError<'a> {
    usage: &'a [u8],
    data: &'a [u8],
}

fn parse_response(input: &[u8]) -> Result<Result<&[u8], CommandError>, NetworkError> {
    defmt::debug!("parse response: {}", Debug2Format(input.as_bstr()));

    let input = input
        .strip_prefix(b"\r\n")
        .ok_or(NetworkError::Parse(ParseError::ResponsePrefix))?;
    let input = input
        .strip_suffix(b"\r\n> ")
        .ok_or(NetworkError::Parse(ParseError::ResponseSuffix))?;

    let index = input
        .rfind(b"\r\n")
        .ok_or(NetworkError::Parse(ParseError::Delimiter))?;

    let data = &input[..index];
    let status = &input[index + 2..];

    Ok(match status {
        b"OK" => Ok(data),
        usage => Err(CommandError { usage, data }),
    })
}

#[derive(Copy, Clone, defmt::Format)]
pub struct Ipv4Addr(pub u8, pub u8, pub u8, pub u8);

impl FromStr for Ipv4Addr {
    type Err = NetworkError;

    fn from_str(input: &str) -> Result<Self, NetworkError> {
        let mut iter = input.splitn(4, '.');
        let a = iter
            .next()
            .ok_or(NetworkError::Parse(ParseError::Delimiter))?
            .parse::<u8>()?;
        let b = iter
            .next()
            .ok_or(NetworkError::Parse(ParseError::Delimiter))?
            .parse::<u8>()?;
        let c = iter
            .next()
            .ok_or(NetworkError::Parse(ParseError::Delimiter))?
            .parse::<u8>()?;
        let d = iter
            .next()
            .ok_or(NetworkError::Parse(ParseError::Delimiter))?
            .parse::<u8>()?;
        Ok(Ipv4Addr(a, b, c, d))
    }
}

impl Display for Ipv4Addr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}.{}.{}.{}", self.0, self.1, self.2, self.3)
    }
}

fn parse_join_message(input: &[u8]) -> Result<(ArrayVec<u8, 32>, Ipv4Addr), NetworkError> {
    // "[JOIN   ] SSID,192.168.1.170,0,0"
    let input = input
        .strip_prefix(b"[JOIN   ] ")
        .ok_or(NetworkError::Parse(ParseError::ResponsePrefix))?;

    let mut iter = input.rfind_iter(b",");
    if let (Some(_), Some(ip_end), Some(ssid_end)) = (iter.next(), iter.next(), iter.next()) {
        let ssid = &input[..ssid_end];
        let ip = &input[ssid_end + 1..ip_end];
        Ok((
            ArrayVec::try_from(ssid)?,
            core::str::from_utf8(ip)?.parse()?,
        ))
    } else {
        Err(NetworkError::Parse(ParseError::Delimiter))
    }
}

fn parse_async_message(input: &[u8]) -> Result<&[u8], NetworkError> {
    let input = input
        .strip_prefix(b"[SOMA]")
        .ok_or(NetworkError::Parse(ParseError::ResponsePrefix))?;
    input
        .strip_suffix(b"[EOMA]")
        .ok_or(NetworkError::Parse(ParseError::ResponseSuffix))
}
