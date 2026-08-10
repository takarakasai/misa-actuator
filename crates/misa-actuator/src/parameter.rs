//! One motor setting, in a shape every driver family can be flattened into.
//!
//! The three families store settings in genuinely different ways — RobStride
//! has a classic indexed space plus a bulk table whose rows name themselves,
//! DAMIAO has numbered registers with three model-dependent layouts above
//! `0x25`, MyActuator has a scatter of per-topic commands plus an indexed
//! block. Nothing useful is gained by modelling that variety in the UI, which
//! only ever wants to show a name, a value and where it came from.
//!
//! So this is deliberately lossy: a driver flattens its own space into a list
//! of these, and the address survives as free text rather than as a number,
//! because `0x2007` and `RID 21` and `0xC0[0x3E]` are not the same kind of
//! thing and pretending otherwise would invite arithmetic on them.

/// A parameter's value, or why it has none.
#[derive(Debug, Clone, PartialEq)]
pub enum ParamValue {
    Float(f32),
    Int(i64),
    Bool(bool),
    Text(String),
    /// The read failed or the motor did not answer. Carried rather than
    /// dropped: "this register did not reply" and "this register is absent
    /// from the list" mean different things to somebody diagnosing a motor.
    Unavailable(String),
}

impl ParamValue {
    pub fn is_available(&self) -> bool {
        !matches!(self, ParamValue::Unavailable(_))
    }
}

/// One setting, as the UI will show it.
#[derive(Debug, Clone, PartialEq)]
pub struct Parameter {
    /// Section heading, e.g. `"control loop gains"`. Drivers group their own
    /// parameters; the UI does not try to impose a common taxonomy, because
    /// the families genuinely do not share one.
    pub group: &'static str,
    /// The name the motor or its manual uses. Kept verbatim — matching it
    /// against a vendor tool's display is the main reason to read this tab.
    pub name: String,
    pub value: ParamValue,
    /// SI unit, or empty when the value is a code or a flag.
    pub unit: &'static str,
    /// Where it lives, as the driver's own notation: `"0x2007"`, `"RID 21"`,
    /// `"0xC0[0x3E]"`. For finding the same value in a vendor tool or manual.
    pub address: String,
    /// True when this came from a reverse-engineered address space rather
    /// than a documented one. The UI marks these: a value read from an
    /// address nobody published is worth less trust than one from a manual.
    pub undocumented: bool,
}

impl Parameter {
    pub fn new(
        group: &'static str,
        name: impl Into<String>,
        value: ParamValue,
        unit: &'static str,
        address: impl Into<String>,
    ) -> Self {
        Self {
            group,
            name: name.into(),
            value,
            unit,
            address: address.into(),
            undocumented: false,
        }
    }

    /// Mark as coming from a reverse-engineered address space.
    pub fn undocumented(mut self) -> Self {
        self.undocumented = true;
        self
    }

    pub fn float(
        group: &'static str,
        name: impl Into<String>,
        v: f32,
        unit: &'static str,
        address: impl Into<String>,
    ) -> Self {
        Self::new(group, name, ParamValue::Float(v), unit, address)
    }

    pub fn int(
        group: &'static str,
        name: impl Into<String>,
        v: i64,
        unit: &'static str,
        address: impl Into<String>,
    ) -> Self {
        Self::new(group, name, ParamValue::Int(v), unit, address)
    }

    pub fn text(
        group: &'static str,
        name: impl Into<String>,
        v: impl Into<String>,
        address: impl Into<String>,
    ) -> Self {
        Self::new(group, name, ParamValue::Text(v.into()), "", address)
    }

    /// A parameter the motor would not report, with the reason.
    pub fn unavailable(
        group: &'static str,
        name: impl Into<String>,
        why: impl Into<String>,
        unit: &'static str,
        address: impl Into<String>,
    ) -> Self {
        Self::new(
            group,
            name,
            ParamValue::Unavailable(why.into()),
            unit,
            address,
        )
    }
}

/// Group names in first-seen order, for rendering sections without the UI
/// having to know any driver's taxonomy.
pub fn groups_in_order(params: &[Parameter]) -> Vec<&'static str> {
    let mut seen: Vec<&'static str> = Vec::new();
    for p in params {
        if !seen.contains(&p.group) {
            seen.push(p.group);
        }
    }
    seen
}

/// Convenience for drivers: turn a `Result` into a value or an explained gap,
/// so one dead register does not abort a whole dump.
pub fn or_unavailable<T, E: std::fmt::Display>(
    r: Result<T, E>,
    to_value: impl FnOnce(T) -> ParamValue,
) -> ParamValue {
    match r {
        Ok(v) => to_value(v),
        Err(e) => ParamValue::Unavailable(format!("{e}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn groups_keep_the_order_the_driver_emitted_them_in() {
        let params = vec![
            Parameter::float("limits", "uv", 15.0, "V", "RID 0"),
            Parameter::float("gains", "kp", 1.0, "", "RID 1"),
            Parameter::float("limits", "ov", 32.0, "V", "RID 2"),
        ];
        // "limits" must not jump below "gains" just because a second limits
        // row appeared later; a dump should read in the order the driver
        // chose to present it.
        assert_eq!(groups_in_order(&params), vec!["limits", "gains"]);
    }

    #[test]
    fn an_unavailable_reading_is_distinguishable_from_a_missing_row() {
        let p = Parameter::unavailable("x", "vbus", "timeout", "V", "RID 3");
        assert!(!p.value.is_available());
        assert_eq!(p.value, ParamValue::Unavailable("timeout".to_string()));
    }
}
