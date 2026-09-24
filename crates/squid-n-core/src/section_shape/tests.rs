use super::*;

#[test]
fn rc_beam_shape_has_section_properties() {
    let shape = SectionShape::RcBeamRect {
        b: 300.0,
        d: 500.0,
        rebar: RcBeamRebar {
            main_dia: 22.0,
            top: vec![3],
            bottom: vec![4],
            cover: 40.0,
            stirrup: BeamStirrup {
                dia: 10.0,
                pitch: 150.0,
                legs: 2,
            },
        },
    };
    assert_eq!(shape.calc_area(), 150_000.0);
    assert!(
        shape
            .to_section(crate::ids::SectionId(0), "RCB".into())
            .area
            > 0.0
    );
}

#[test]
fn rc_column_shapes_are_concrete_like() {
    let shape = SectionShape::RcColumnCircle {
        d: 600.0,
        rebar: RcCircleColumnRebar {
            main_dia: 19.0,
            count: 12,
            cover: 40.0,
            hoop: CircleColumnHoop {
                dia: 10.0,
                pitch: 150.0,
            },
        },
    };
    assert!(shape.is_concrete_like());
    assert!(shape.validate_rebar().is_ok());
}
