fn main() -> Result<(), Box<dyn std::error::Error>> {
    mt_sea::init_logging();
    let mut var = rat::rfalse();
    let mut var3 = rat::rfalse();
    let mut var4 = rat::rfalse();

    rat::init("testRat2", None, None)?;

    rat::bacon("var1", &mut var, mt_sea::VariableType::U8)?;
    assert!(var == rat::rtrue());

    rat::bacon("var3", &mut var3, mt_sea::VariableType::U8)?;
    assert!(var3 == rat::rfalse());

    rat::bacon("var4", &mut var4, mt_sea::VariableType::U8)?;
    assert!(var4 == rat::rfalse());

    rat::deinit()?;

    let green = "\x1b[32m";
    let reset = "\x1b[0m";
    println!("\n{green}Success!{reset}");
    Ok(())
}
