macro_rules! id_newtype {
    ($name:ident) => {
        #[derive(
            Clone,
            Copy,
            PartialEq,
            Eq,
            Hash,
            PartialOrd,
            Ord,
            Debug,
            serde::Serialize,
            serde::Deserialize,
        )]
        pub struct $name(pub u32);
        impl $name {
            pub fn index(self) -> usize {
                self.0 as usize
            }
        }
    };
}

id_newtype!(NodeId);
id_newtype!(ElemId);
id_newtype!(StoryId);
id_newtype!(SlabId);
id_newtype!(SectionId);
id_newtype!(MaterialId);
id_newtype!(LoadCaseId);
id_newtype!(VibrationCaseId);
id_newtype!(LumpedVibrationCaseId);
id_newtype!(SecondaryMemberId);

impl Default for SecondaryMemberId {
    /// 旧スキーマからの読込時に `#[serde(default)]` で補完される値（0 固定）。
    /// `Model::migrate_secondary_member_ids` でインデックスと整合させること。
    fn default() -> Self {
        SecondaryMemberId(0)
    }
}
