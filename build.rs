use std::fmt::Write;

fn main() {
    println!("cargo::rerun-if-changed=wifi-networks.csv");
    let data =
        std::fs::read_to_string("wifi-networks.csv").expect("failed to read wifi-networks.csv");

    let networks = data
        .lines()
        .flat_map(|line| line.split_once(','))
        .collect::<Vec<_>>();

    let mut code = String::new();
    write!(
        &mut code,
        "const WIFI_NETWORKS: [(&[u8], &[u8]); {}] = [\n",
        networks.len()
    )
    .unwrap();
    for (network, password) in networks {
        write!(
            &mut code,
            "    (b\"{}\", b\"{}\"),\n",
            network.as_bytes().escape_ascii(),
            password.as_bytes().escape_ascii()
        )
        .unwrap();
    }
    write!(&mut code, "];\n").unwrap();

    std::fs::write(
        std::path::PathBuf::from(std::env::var_os("OUT_DIR").unwrap()).join("wifi-networks.rs"),
        code,
    )
    .expect("failed to write wifi-networks.rs");
}
