fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut var = rat::rtrue();

    rat::init("testRat1", None)?;

    rat::bacon("var1", &mut var)?;

    rat::deinit()?;

    assert!(var == rat::rfalse());
    Ok(())
}
