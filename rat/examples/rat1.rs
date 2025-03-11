fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let mut var1 = rat::rtrue();
    let mut var2 = rat::rfalse();
    let mut var3 = rat::rtrue();
    let mut var4 = rat::rtrue();

    rat::init("testRat1", None)?;

    // exists on rat2
    rat::bacon("var1", &mut var1, sea::VariableType::U8)?;
    assert!(var1 == rat::rtrue());

    // does not exist, so should sail
    rat::bacon("var2", &mut var2, sea::VariableType::U8)?;
    assert!(var2 == rat::rfalse());

    rat::bacon("var3", &mut var3, sea::VariableType::U8)?;
    assert!(var3 == rat::rfalse());

    rat::bacon("var4", &mut var4, sea::VariableType::U8)?;
    assert!(var4 == rat::rtrue());

    rat::deinit()?;

    let green = "\x1b[32m";
    let reset = "\x1b[0m";
    println!("\n{green}Success!{reset}");
    Ok(())
}
