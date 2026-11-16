// SPDX-License-Identifier: BSD-3-Clause
//! Summary lines such as `IOMMU gate: 31/31 checks passed`, described by a
//! template with numbered fields.

use std::fmt;

/// A summary line to look for, with the counts it reports.
///
/// The template is literal text with fields that each match one or more
/// ASCII digits:
///
/// - `{passed}`, required once: the number of checks that passed;
/// - `{total}`, optional, at most once: the number of checks that ran;
/// - `{_}`, any number of times: a number that is matched and ignored.
///
/// A field must be followed by the end of the template or by text that does
/// not start with a digit, so that a field always takes every digit in its
/// run, as a regular expression's `\d+` would.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Summary {
    template: String,
    parts: Vec<Part>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Part {
    Text(Vec<u8>),
    Passed,
    Total,
    Ignored,
}

/// A summary found in the output.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Counts {
    /// The text that matched the template.
    pub text: String,
    /// The `{passed}` field.
    pub passed: u64,
    /// The `{total}` field, when the template has one.
    pub total: Option<u64>,
}

/// Why a summary template was refused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TemplateError {
    /// A brace that does not belong to `{passed}`, `{total}` or `{_}`.
    UnknownField(String),
    /// No `{passed}` field.
    NoPassed,
    /// `{passed}` or `{total}` more than once.
    Repeated(&'static str),
    /// A field followed directly by a digit or by another field.
    Ambiguous,
}

impl fmt::Display for TemplateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownField(text) => {
                write!(
                    f,
                    "unknown field at \"{text}\": use {{passed}}, {{total}} or {{_}}"
                )
            }
            Self::NoPassed => write!(f, "the template has no {{passed}} field"),
            Self::Repeated(field) => write!(f, "{{{field}}} appears more than once"),
            Self::Ambiguous => {
                write!(f, "a field is followed by a digit or another field")
            }
        }
    }
}

impl std::error::Error for TemplateError {}

impl Summary {
    /// Parses a template such as `IOMMU gate: {passed}/{total} checks passed`.
    pub fn parse(template: &str) -> Result<Self, TemplateError> {
        let mut parts = Vec::new();
        let mut text = Vec::new();
        let mut rest = template;
        while let Some(c) = rest.chars().next() {
            if c == '{' {
                let field = [
                    ("{passed}", Part::Passed),
                    ("{total}", Part::Total),
                    ("{_}", Part::Ignored),
                ]
                .into_iter()
                .find(|(name, _)| rest.starts_with(name));
                let Some((name, part)) = field else {
                    return Err(TemplateError::UnknownField(rest.to_owned()));
                };
                if !text.is_empty() {
                    parts.push(Part::Text(std::mem::take(&mut text)));
                }
                parts.push(part);
                rest = &rest[name.len()..];
            } else if c == '}' {
                return Err(TemplateError::UnknownField(rest.to_owned()));
            } else {
                text.extend_from_slice(&rest.as_bytes()[..c.len_utf8()]);
                rest = &rest[c.len_utf8()..];
            }
        }
        if !text.is_empty() {
            parts.push(Part::Text(text));
        }

        for (field, name) in [(Part::Passed, "passed"), (Part::Total, "total")] {
            match parts.iter().filter(|part| **part == field).count() {
                0 if field == Part::Passed => return Err(TemplateError::NoPassed),
                0 | 1 => {}
                _ => return Err(TemplateError::Repeated(name)),
            }
        }
        for pair in parts.windows(2) {
            let next_is_digit = match &pair[1] {
                Part::Text(text) => text[0].is_ascii_digit(),
                _ => true,
            };
            if !matches!(pair[0], Part::Text(_)) && next_is_digit {
                return Err(TemplateError::Ambiguous);
            }
        }
        Ok(Self {
            template: template.to_owned(),
            parts,
        })
    }

    /// Finds the first match in `output`, as a regular expression search
    /// would.
    pub fn find(&self, output: &[u8]) -> Option<Counts> {
        (0..=output.len()).find_map(|start| self.match_at(output, start))
    }

    fn match_at(&self, output: &[u8], start: usize) -> Option<Counts> {
        let mut at = start;
        let mut passed = None;
        let mut total = None;
        for part in &self.parts {
            match part {
                Part::Text(text) => {
                    if !output[at..].starts_with(text) {
                        return None;
                    }
                    at += text.len();
                }
                field => {
                    let digits = output[at..]
                        .iter()
                        .take_while(|b| b.is_ascii_digit())
                        .count();
                    if digits == 0 {
                        return None;
                    }
                    // ASCII digits are valid UTF-8; only overflow can fail.
                    let number = std::str::from_utf8(&output[at..at + digits])
                        .ok()?
                        .parse::<u64>()
                        .ok()?;
                    match field {
                        Part::Passed => passed = Some(number),
                        Part::Total => total = Some(number),
                        _ => {}
                    }
                    at += digits;
                }
            }
        }
        Some(Counts {
            text: String::from_utf8_lossy(&output[start..at]).into_owned(),
            passed: passed?,
            total,
        })
    }
}

impl fmt::Display for Summary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.template)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_both_counts() {
        let summary = Summary::parse("gate: {passed}/{total} checks passed").unwrap();
        let counts = summary.find(b"x\ngate: 30/31 checks passed\r\n").unwrap();
        assert_eq!(counts.text, "gate: 30/31 checks passed");
        assert_eq!((counts.passed, counts.total), (30, Some(31)));
    }

    #[test]
    fn ignored_field_must_still_match() {
        let summary = Summary::parse("ok. {passed} passed. {_} disabled.").unwrap();
        assert_eq!(summary.find(b"ok. 3 passed. disabled."), None);
        let counts = summary.find(b"ok. 3 passed. 12 disabled.").unwrap();
        assert_eq!((counts.passed, counts.total), (3, None));
    }

    #[test]
    fn first_match_wins() {
        let summary = Summary::parse("n={passed};").unwrap();
        assert_eq!(summary.find(b"n=; n=7; n=8;").unwrap().passed, 7);
    }

    #[test]
    fn field_takes_the_whole_digit_run() {
        let summary = Summary::parse("{passed}/{total}").unwrap();
        let counts = summary.find(b"run 120/121").unwrap();
        assert_eq!((counts.passed, counts.total), (120, Some(121)));
    }

    #[test]
    fn refuses_bad_templates() {
        use TemplateError::*;
        let cases = [
            ("checks passed", NoPassed),
            ("{passed} {passed}", Repeated("passed")),
            ("{passed}/{total}/{total}", Repeated("total")),
            ("{passed}{total}", Ambiguous),
            ("{passed}0", Ambiguous),
            ("{count} passed", UnknownField("{count} passed".into())),
            ("{passed} }", UnknownField("}".into())),
        ];
        for (template, error) in cases {
            assert_eq!(Summary::parse(template), Err(error), "{template}");
        }
    }
}
