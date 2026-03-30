use std::env;
use std::path::{Path, PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=LIVOX_SDK2_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=LIVOX_SDK2_LIB_DIR");

    let include_dir = env::var_os("LIVOX_SDK2_INCLUDE_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/usr/local/include"));
    let lib_dir = env::var_os("LIVOX_SDK2_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/usr/local/lib"));

    let header = include_dir.join("livox_lidar_api.h");
    let dylib = candidate_paths(&lib_dir, "livox_lidar_sdk_shared");

    if header.exists() && dylib.iter().any(|path| path.exists()) {
        println!("cargo:rustc-check-cfg=cfg(livox_sdk2_linked)");
        println!("cargo:rustc-cfg=livox_sdk2_linked");
        println!("cargo:rustc-link-search=native={}", lib_dir.display());
        println!("cargo:rustc-link-lib=dylib=livox_lidar_sdk_shared");
    } else {
        println!("cargo:rustc-check-cfg=cfg(livox_sdk2_linked)");
        println!(
            "cargo:warning=Livox-SDK2 not found under {} / {}; building stub backend",
            include_dir.display(),
            lib_dir.display()
        );
    }
}

fn candidate_paths(lib_dir: &Path, base: &str) -> Vec<PathBuf> {
    [
        format!("lib{base}.so"),
        format!("lib{base}.dylib"),
        format!("lib{base}.a"),
    ]
    .into_iter()
    .map(|name| lib_dir.join(name))
    .collect()
}
