fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let mut var = rat::rfalse();
    let mut var3 = rat::rfalse();

    rat::init("testRat2", None)?;

    rat::bacon("var1", &mut var, sea::VariableType::U8)?;
    assert!(var == rat::rtrue());

    rat::bacon("var3", &mut var3, sea::VariableType::U8)?;
    assert!(var3 == rat::rfalse());

    rat::deinit()?;

    let green = "\x1b[32m";
    let reset = "\x1b[0m";
    println!("\n{green}Success!{reset}");
    Ok(())
}
