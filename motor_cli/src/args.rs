use std::collections::HashMap;

pub fn parse_args() -> HashMap<String, String> {
    let mut out = HashMap::new();
    let mut it = std::env::args().skip(1).peekable();
    while let Some(k) = it.next() {
        if k == "-h" || k == "--help" || k == "help" {
            out.insert("help".to_string(), "1".to_string());
            continue;
        }
        if k == "-v" || k == "--version" {
            out.insert("version".to_string(), "1".to_string());
            continue;
        }
        // Ignore common cargo-only flag if user accidentally passes it to binary.
        if k == "--release" {
            continue;
        }
        if !k.starts_with("--") {
            // Accept the Python CLI style `motor_cli scan --vendor ...` as a
            // shorthand for the Rust CLI's historical `--mode scan` form.
            if !out.contains_key("mode") && is_mode_word(&k) {
                out.insert("mode".to_string(), k);
            }
            continue;
        }
        let key = k.trim_start_matches("--").to_string();
        match it.peek() {
            Some(v) if !v.starts_with("--") => {
                if let Some(val) = it.next() {
                    out.insert(key, val);
                }
            }
            _ => {
                out.insert(key, "1".to_string());
            }
        }
    }
    out
}

fn is_mode_word(s: &str) -> bool {
    matches!(
        s,
        "scan"
            | "enable"
            | "disable"
            | "vel"
            | "mit"
            | "pos-vel"
            | "force-pos"
            | "set-zero"
            | "read-param"
            | "write-param"
    )
}

pub fn get_str(args: &HashMap<String, String>, key: &str, default: &str) -> String {
    args.get(key)
        .cloned()
        .unwrap_or_else(|| default.to_string())
}

pub fn get_f32(args: &HashMap<String, String>, key: &str, default: f32) -> Result<f32, String> {
    match args.get(key) {
        Some(v) => v
            .parse::<f32>()
            .map_err(|e| format!("invalid --{key}: {e}")),
        None => Ok(default),
    }
}

pub fn get_i16(args: &HashMap<String, String>, key: &str, default: i16) -> Result<i16, String> {
    match args.get(key) {
        Some(v) => v
            .parse::<i16>()
            .map_err(|e| format!("invalid --{key}: {e}")),
        None => Ok(default),
    }
}

pub fn get_u64(args: &HashMap<String, String>, key: &str, default: u64) -> Result<u64, String> {
    match args.get(key) {
        Some(v) => v
            .parse::<u64>()
            .map_err(|e| format!("invalid --{key}: {e}")),
        None => Ok(default),
    }
}

pub fn parse_u16_hex_or_dec(s: &str, key: &str) -> Result<u16, String> {
    if let Some(hex) = s.strip_prefix("0x") {
        u16::from_str_radix(hex, 16).map_err(|e| format!("invalid --{key}: {e}"))
    } else {
        s.parse::<u16>()
            .map_err(|e| format!("invalid --{key}: {e}"))
    }
}

pub fn get_u16_hex_or_dec(
    args: &HashMap<String, String>,
    key: &str,
    default: u16,
) -> Result<u16, String> {
    match args.get(key) {
        Some(v) => parse_u16_hex_or_dec(v, key),
        None => Ok(default),
    }
}

pub fn get_opt_u16_hex_or_dec(
    args: &HashMap<String, String>,
    key: &str,
) -> Result<Option<u16>, String> {
    match args.get(key) {
        Some(v) => Ok(Some(parse_u16_hex_or_dec(v, key)?)),
        None => Ok(None),
    }
}

pub fn get_u16_list_hex_or_dec(
    args: &HashMap<String, String>,
    key: &str,
    default: &[u16],
) -> Result<Vec<u16>, String> {
    let Some(raw) = args.get(key) else {
        return Ok(default.to_vec());
    };
    let mut out = Vec::new();
    for part in raw.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        let value = parse_u16_hex_or_dec(part, key)?;
        if !out.contains(&value) {
            out.push(value);
        }
    }
    if out.is_empty() {
        return Err(format!("invalid --{key}: expected comma-separated id list"));
    }
    Ok(out)
}

pub fn print_help() {
    println!(
        "motor_cli\n\
Usage:\n\
  motor_cli -h | --help\n\
  motor_cli -v | --version\n\
  motor_cli --vendor damiao --mode scan --start-id 1 --end-id 16\n\
  motor_cli --vendor damiao --mode enable --motor-id 0x01 --feedback-id 0x11\n\n\
Behavior:\n\
  no arguments: print this help (no motor command is sent)\n\n\
Mode shorthand:\n\
  A bare mode word (for example `scan`) is accepted as shorthand for `--mode scan`.\n\n\
CLI form:\n\
  motor_cli --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \\\n\
    --mode mit --pos 0 --vel 0 --kp 2 --kd 1 --tau 0 --loop 200 --dt-ms 20\n\n\
Vendors:\n\
  --vendor damiao    default; this fork only supports Damiao\n\n\
Damiao modes:\n\
  --mode scan | enable | disable | mit | pos-vel | vel | force-pos | read-param | write-param\n\n\
Common args:\n\
  --transport   auto|socketcan|socketcanfd|dm-serial|dm-device (default auto)\n\
  --channel      default can0\n\
  --serial-port  default /dev/ttyACM0 (used when --transport dm-serial)\n\
  --serial-baud  default 921600 (used when --transport dm-serial)\n\
  --dm-device-type  usb2canfd|usb2canfd-dual|linkx4c, default usb2canfd-dual (used when --transport dm-device)\n\
  --dm-channel      SDK channel number: usb2canfd=0, usb2canfd-dual=0|1, linkx4c=0|1|2|3 (control default 0; scan omitted scans all)\n\
  --model        Damiao model, default 4340\n\
  --motor-id     default 0x01\n\
  --feedback-id  default 0x11\n\
  --loop         send cycles, default 1\n\
  --dt-ms        period ms, default 20\n\
  --ensure-mode  1/0, default 1\n\n\
ID change support by vendor:\n\
  Damiao: supported (`--set-motor-id` and optional `--set-feedback-id`)\n\n\
Damiao extras:\n\
  --verify-model 1/0, default 1\n\
  --verify-timeout-ms  default 500\n\
  --verify-tol   default 0.2\n\
  --set-motor-id <id> --set-feedback-id <id> --store 1/0 --verify-id 1/0\n\
  --param-id <hex|dec>      for read-param / write-param\n\
  --param-value <number>    for write-param\n\
  --type u32|f32            for read-param / write-param (default f32)\n\
  --store 1/0               for write-param store/save, default 0; for zero-exp, default 1\n\
  --start-id <hex|dec>      for scan, default 1\n\
  --end-id <hex|dec>        for scan, default 255\n\
\n\
Run-mode effective arguments:\n\
  Damiao mit:        --pos --vel --kp --kd --tau\n\
  Damiao pos-vel:    --pos --vlim\n\
  Damiao vel:        --vel\n\
  Damiao force-pos:  --pos --vlim --ratio\n\
\n\
Examples:\n\
  motor_cli --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \\\n\
    --mode read-param --param-id 10\n\
\n\
  motor_cli --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \\\n\
    --mode mit --ensure-mode 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20\n"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_u16_hex_or_dec_supports_both_formats() {
        assert_eq!(parse_u16_hex_or_dec("0x10", "x").expect("hex"), 16);
        assert_eq!(parse_u16_hex_or_dec("255", "x").expect("dec"), 255);
    }

    #[test]
    fn parse_u16_hex_or_dec_rejects_invalid_values() {
        assert!(parse_u16_hex_or_dec("0xZZ", "x").is_err());
        assert!(parse_u16_hex_or_dec("-1", "x").is_err());
    }

    #[test]
    fn get_u16_hex_or_dec_uses_default_when_missing() {
        let args = HashMap::new();
        assert_eq!(
            get_u16_hex_or_dec(&args, "motor-id", 0x01).expect("default"),
            0x01
        );
    }

    #[test]
    fn recognizes_positional_mode_words() {
        assert!(is_mode_word("scan"));
        assert!(is_mode_word("read-param"));
        assert!(!is_mode_word("can0"));
    }

    #[test]
    fn get_u16_list_hex_or_dec_parses_and_deduplicates() {
        let mut args = HashMap::new();
        args.insert("feedback-ids".to_string(), "0xFD, 0xFF,253".to_string());
        assert_eq!(
            get_u16_list_hex_or_dec(&args, "feedback-ids", &[0xAA]).expect("list"),
            vec![0xFD, 0xFF]
        );
    }
}
