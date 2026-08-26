use crate::format::*;

#[test]
fn a_record_prints_the_level_first_and_the_source_second() {
    let record = Record::new(LogLevel::Info, "Pelorus is ready").with_source("Pelorus");
    assert_eq!(
        record.console_line(false),
        "[INFO] [Pelorus] Pelorus is ready"
    );
    assert_eq!(
        Record::new(LogLevel::Warn, "no fix").console_line(false),
        "[WARN] no fix"
    );
}

/// The source tag is what a parent asked for. A process does not choose it
/// announces about itself when nobody is listening.
#[test]
fn the_source_tag_appears_only_in_the_tagged_formats() {
    let record = Record::new(LogLevel::Info, "started").with_source("Minot");
    assert_eq!(record.line(Format::Text, false), "[INFO] started");
    assert_eq!(record.line(Format::Tagged, false), "[INFO] [Minot] started");
}

#[test]
fn a_tagged_line_round_trips_through_the_parser() {
    let record = Record::new(LogLevel::Warn, "bag is short")
        .with_source("Minot")
        .with_target("bagread");
    let parsed = parse(&record.line(Format::Tagged, false));

    assert_eq!(parsed.level, LogLevel::Warn);
    assert_eq!(parsed.source.as_deref(), Some("Minot"));
    assert_eq!(parsed.target.as_deref(), Some("bagread"));
    assert_eq!(parsed.message, "bag is short");
}

/// JSON keeps what text has to re-derive: a message with a colon in it is
/// stays separate from the target and provides the explicit level.
#[cfg(feature = "json")]
#[test]
fn a_json_line_round_trips_with_every_field_intact() {
    let record = Record::new(LogLevel::Error, "no such topic: /velodyne_points")
        .with_source("Minot")
        .with_target("app");
    let parsed = parse(&record.line(Format::Json, false));

    assert_eq!(parsed, record);
}

/// The level word appearing anywhere in the line is the last resort, and must
/// not win over a tag the child actually wrote.
#[test]
fn a_tagged_level_beats_the_guess() {
    assert_eq!(
        parse("[INFO] could not parse ERROR threshold").level,
        LogLevel::Info
    );
    assert_eq!(
        parse("could not parse ERROR threshold").level,
        LogLevel::Error
    );
}

/// A coloured tag from a child would otherwise hide the level from the parser
/// and end up printed twice.
#[test]
fn colour_codes_do_not_hide_the_level() {
    let parsed = parse("\u{1b}[36m[INFO]\u{1b}[0m Using ZID: 9d687664");

    assert_eq!(parsed.level, LogLevel::Info);
    assert_eq!(parsed.message, "Using ZID: 9d687664");
}

/// `[Minot] minot::app:` says Minot twice. The module is what the tag does
/// not already say.
#[test]
fn a_target_that_repeats_its_source_is_trimmed() {
    let parsed = parse("[INFO] [Minot] minot::app: read bag");
    assert_eq!(parsed.target.as_deref(), Some("app"));
    assert_eq!(parsed.pane_message(), "[Minot] app: read bag");

    let parsed = parse("[INFO] [Minot] minot: compiled in 243.292µs");
    assert_eq!(parsed.target, None);
    assert_eq!(parsed.pane_message(), "[Minot] compiled in 243.292µs");
}

/// Bracketed text that is not a tag, and colons that are not a target, belong
/// to the message.
#[test]
fn ordinary_text_is_not_mistaken_for_tags() {
    let parsed = parse("[INFO] Zenoh can be reached at: tcp/[fe80::1]:49507");
    assert_eq!(parsed.source, None);
    assert_eq!(parsed.target, None);
    assert_eq!(
        parsed.message,
        "Zenoh can be reached at: tcp/[fe80::1]:49507"
    );

    let parsed = parse("[WARN] north/e42b4c22:10:5881 Route final reply: Query not found!");
    assert_eq!(parsed.target, None);
    assert_eq!(
        parsed.message,
        "north/e42b4c22:10:5881 Route final reply: Query not found!"
    );
}

#[test]
fn relaying_names_the_innermost_process() {
    let own = parse("[INFO] Using ZID: 9d687664").or_source("Pelorus");
    assert_eq!(own.pane_message(), "[Pelorus] Using ZID: 9d687664");

    let relayed = parse("[INFO] [Minot] read bag").or_source("Pelorus");
    assert_eq!(relayed.pane_message(), "[Minot] read bag");
}

#[test]
fn escape_sequences_and_control_characters_are_stripped() {
    assert_eq!(
        sanitize("\u{1b}[33m[WARN]\u{1b}[0m Query\tnot\nfound!\u{7}".to_owned()),
        "[WARN] Query not found!"
    );
    assert_eq!(sanitize("Loop detected!".to_owned()), "Loop detected!");
}

/// The chain this crate exists for: Minot writes a record, Pelorus reads it
/// and re-emits it as its own, and Polarstern reads that. The level and the
/// name of the process that first wrote the line have to survive both hops,
/// and nothing may be said twice.
#[cfg(feature = "json")]
#[test]
fn a_record_survives_two_relays() {
    // Minot, run by a parent that asked for records.
    let minot = Record::new(LogLevel::Warn, "bag ends early")
        .with_source("Minot")
        .with_target("minot::app")
        .line(Format::Json, false);

    // Pelorus reads it, and logs it under a target of its own.
    let at_pelorus = parse(&minot);
    assert_eq!(at_pelorus.level, LogLevel::Warn);
    assert_eq!(at_pelorus.pane_message(), "[Minot] app: bag ends early");
    let relayed = Record::new(at_pelorus.level, at_pelorus.pane_message())
        .with_source("Pelorus")
        .line(Format::Json, false);

    // Polarstern reads that, and shows it in its pane.
    let at_polarstern = parse(&relayed).or_source("Pelorus");
    assert_eq!(at_polarstern.level, LogLevel::Warn);
    assert_eq!(
        at_polarstern.pane_message(),
        "[Minot] app: bag ends early",
        "the process that wrote the line is the one named, once"
    );
}

/// The same chain over plain text, for a child too old to speak records.
#[test]
fn an_untagged_child_still_comes_through() {
    let at_pelorus = parse("[INFO] Using ZID: 9d687664").or_source("Pelorus");
    let relayed = at_pelorus.line(Format::Tagged, false);
    assert_eq!(relayed, "[INFO] [Pelorus] Using ZID: 9d687664");

    let at_polarstern = parse(&relayed).or_source("Pelorus");
    assert_eq!(
        at_polarstern.pane_message(),
        "[Pelorus] Using ZID: 9d687664"
    );
}

#[cfg(feature = "tui")]
mod tui {
    use crate::buffer::LogBuffer;
    use crate::format::LogLevel;
    use crate::pane::{LogView, wrap};

    #[test]
    fn the_buffer_keeps_the_newest_entries() {
        let buffer = LogBuffer::new("polarstern");
        for index in 0..10_005 {
            buffer.push(LogLevel::Info, "polarstern", format!("line {index}"));
        }
        let entries = buffer.snapshot();

        assert_eq!(entries.len(), 10_000);
        assert_eq!(entries.last().unwrap().message, "line 10004");
    }

    #[test]
    fn foreign_entries_are_told_apart_by_target() {
        let buffer = LogBuffer::new("polarstern");
        buffer.push(LogLevel::Info, "polarstern::config", "mine");
        buffer.push(LogLevel::Info, "pelorus", "theirs");
        let entries = buffer.snapshot();

        assert!(buffer.is_own(&entries[0]));
        assert!(!buffer.is_own(&entries[1]));
    }

    /// A foreign crate is named. Our own module paths use the process label.
    /// line is already known to be ours.
    #[test]
    fn a_console_line_names_only_a_foreign_target() {
        let buffer = LogBuffer::new("polarstern");
        assert_eq!(
            buffer.console_line(LogLevel::Warn, "zenoh::net", "no route"),
            "[WARN] zenoh::net: no route"
        );
        assert_eq!(
            buffer.console_line(LogLevel::Info, "polarstern::tui", "hi"),
            "[INFO] hi"
        );
    }

    /// The first key press must not let the tail scroll out from under the
    /// reader, and reaching the bottom must resume following.
    #[test]
    fn scrolling_leaves_and_rejoins_the_tail() {
        let mut view = LogView::default();
        assert!(view.following());

        view.scroll_up(10);
        view.scroll_up(10);
        view.scroll_down(10);
        view.scroll_down(10);
        assert!(view.following(), "reaching the bottom resumes following");

        view.to_start();
        assert!(!view.following());
        view.to_end();
        assert!(view.following());
    }

    #[test]
    fn paging_up_from_the_tail_stops_at_the_first_entry() {
        let mut view = LogView::default();
        view.page_up(5);
        view.page_up(5);
        view.scroll_up(5);
        assert!(!view.following(), "cannot scroll past the oldest entry");
    }

    #[test]
    fn wrapping_breaks_on_words_and_never_exceeds_the_width() {
        let wrapped = wrap("the quick brown fox jumps", 10, 10);
        assert!(
            wrapped.iter().all(|line| line.chars().count() <= 10),
            "{wrapped:?}"
        );
        assert_eq!(wrapped.join(" "), "the quick brown fox jumps");
    }

    /// The header shortens the first line. The rest get the wider
    /// hanging indent's worth of room.
    #[test]
    fn wrapping_gives_the_first_line_its_own_width() {
        assert_eq!(wrap("aaa bbb ccc ddd", 3, 7), vec!["aaa", "bbb ccc", "ddd"]);
    }

    /// A path or hash with no spaces still has to fit the pane, and a
    /// multi-byte character must not be split down the middle.
    #[test]
    fn wrapping_hard_splits_a_word_longer_than_the_pane() {
        let wrapped = wrap(&"x".repeat(25), 10, 10);
        assert_eq!(wrapped.len(), 3);
        assert!(wrapped.iter().all(|line| line.chars().count() <= 10));

        assert_eq!(wrap(&"µ".repeat(12), 5, 5), vec!["µµµµµ", "µµµµµ", "µµ"]);
    }
}

#[cfg(all(feature = "env", feature = "tui"))]
mod quieted_spec {
    // env_filter comes in with the `tui` feature, which is what parses a spec
    // at runtime. These assert against that same engine.

    use crate::quieted;

    fn allows(spec: &str, target: &str, level: log::Level) -> bool {
        env_filter::Builder::new()
            .parse(spec)
            .build()
            .matches(&log::Record::builder().level(level).target(target).build())
    }

    /// The flood this exists to stop: one best-effort peer walking away makes
    /// every publisher in the process narrate the same expired query.
    #[test]
    fn query_expiry_chatter_is_held_back() {
        for level in ["error", "warn", "info", "debug", "trace"] {
            let spec = quieted(level);
            assert!(
                !allows(&spec, "zenoh::api::session", log::Level::Warn),
                "a late reply for a dropped query still logs at {level}: {spec}"
            );
            assert!(
                !allows(
                    &spec,
                    "zenoh::net::routing::dispatcher::queries",
                    log::Level::Warn
                ),
                "a query deadline still logs at {level}: {spec}"
            );
        }
    }

    /// Held back, still enabled. Nothing in the list is quiet enough to
    /// swallow a real error: a module that has genuinely failed still has
    /// something to say, and silencing it wholesale is how a real fault turns
    /// into a process that stops working with no explanation.
    #[test]
    fn real_transport_errors_still_come_through() {
        let spec = quieted("info");
        for (module, _) in crate::QUIET_ZENOH {
            assert!(
                allows(&spec, module, log::Level::Error),
                "a real error from {module} would go unreported: {spec}"
            );
        }
        assert!(allows(&spec, "zenoh_transport", log::Level::Warn));
    }

    /// A cap, never a floor: the level that asked for the least must not get
    /// transport chatter handed back to it.
    #[test]
    fn a_quieter_level_is_never_made_louder() {
        let spec = quieted("error");
        assert!(!allows(&spec, "zenoh", log::Level::Warn), "{spec}");
        assert!(
            !allows(&spec, "zenoh_transport", log::Level::Warn),
            "{spec}"
        );
    }

    /// A full spec is capped just the same, since a process may be handed one
    /// through the environment with its configured level.
    #[test]
    fn a_full_spec_is_capped_without_losing_what_it_said() {
        let spec = quieted("info,polarstern=debug");
        assert!(allows(&spec, "polarstern", log::Level::Debug), "{spec}");
        assert!(
            !allows(&spec, "zenoh::api::session", log::Level::Warn),
            "{spec}"
        );
    }

    #[test]
    fn the_applications_own_records_are_untouched() {
        let spec = quieted("info");
        assert!(allows(&spec, "polarstern", log::Level::Info));
        assert!(allows(&spec, "pelorus", log::Level::Info));
    }
}
