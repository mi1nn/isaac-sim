fn main() {
    // Monitor the content directory for changes
    println!("cargo:rerun-if-changed=content");
}
