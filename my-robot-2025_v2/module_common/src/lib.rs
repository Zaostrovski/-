
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "op")]
pub enum IncMsgDcModule {
    SetPwmA{pwm: f32},
    SetPwmB{pwm: f32},
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "op")]
pub enum OutMsgDcModule{
    Ok,
}