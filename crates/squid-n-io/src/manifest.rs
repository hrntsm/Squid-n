#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Manifest {
    pub schema_version: u32,
    pub units: String,
    pub created_by: String,
    pub entries: Vec<EntryHash>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EntryHash {
    pub name: String,
    pub sha256: String,
}
