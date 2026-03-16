use rkyv::string::ArchivedString;
fn main() {
    println!(
        "ArchivedString size: {}, align: {}",
        std::mem::size_of::<ArchivedString>(),
        std::mem::align_of::<ArchivedString>()
    );
}
