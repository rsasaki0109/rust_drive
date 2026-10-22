//! Reject nonfinite numeric values before JSON can replace them with null.
use serde::{Serialize, Serializer, ser};
use std::fmt;

pub(super) struct FiniteNumbers;
type Check = Result<(), serde_json::Error>;

macro_rules! accept_scalar {
    ($($method:ident: $kind:ty),+ $(,)?) => {
        $(fn $method(self, _value: $kind) -> Check { Ok(()) })+
    };
}

impl Serializer for FiniteNumbers {
    type Ok = ();
    type Error = serde_json::Error;
    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    accept_scalar! {
        serialize_bool: bool,
        serialize_i8: i8,
        serialize_i16: i16,
        serialize_i32: i32,
        serialize_i64: i64,
        serialize_i128: i128,
        serialize_u8: u8,
        serialize_u16: u16,
        serialize_u32: u32,
        serialize_u64: u64,
        serialize_u128: u128,
        serialize_char: char,
        serialize_str: &str,
        serialize_bytes: &[u8],
    }

    fn serialize_f32(self, value: f32) -> Check {
        self.serialize_f64(f64::from(value))
    }
    fn serialize_f64(self, value: f64) -> Check {
        if value.is_finite() {
            Ok(())
        } else {
            Err(ser::Error::custom("non-finite numeric value in sensor log"))
        }
    }
    fn serialize_none(self) -> Check {
        Ok(())
    }
    fn serialize_some<T: Serialize + ?Sized>(self, value: &T) -> Check {
        value.serialize(self)
    }
    fn serialize_unit(self) -> Check {
        Ok(())
    }
    fn serialize_unit_struct(self, _name: &'static str) -> Check {
        Ok(())
    }
    fn serialize_unit_variant(
        self,
        _name: &'static str,
        _index: u32,
        _variant: &'static str,
    ) -> Check {
        Ok(())
    }
    fn serialize_newtype_struct<T: Serialize + ?Sized>(
        self,
        _name: &'static str,
        value: &T,
    ) -> Check {
        value.serialize(self)
    }
    fn serialize_newtype_variant<T: Serialize + ?Sized>(
        self,
        _name: &'static str,
        _index: u32,
        _variant: &'static str,
        value: &T,
    ) -> Check {
        value.serialize(self)
    }
    fn serialize_seq(self, _len: Option<usize>) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn serialize_tuple(self, _len: usize) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn serialize_tuple_struct(self, _name: &'static str, _len: usize) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn serialize_tuple_variant(
        self,
        _name: &'static str,
        _index: u32,
        _variant: &'static str,
        _len: usize,
    ) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn serialize_map(self, _len: Option<usize>) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn serialize_struct(self, _name: &'static str, _len: usize) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn serialize_struct_variant(
        self,
        _name: &'static str,
        _index: u32,
        _variant: &'static str,
        _len: usize,
    ) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn collect_str<T: fmt::Display + ?Sized>(self, _value: &T) -> Check {
        Ok(())
    }
}

macro_rules! check_elements {
    ($trait:ident, $method:ident $(, $key:ident)?) => {
        impl ser::$trait for FiniteNumbers {
            type Ok = ();
            type Error = serde_json::Error;
            fn $method<T: Serialize + ?Sized>(
                &mut self,
                $($key: &'static str,)?
                value: &T,
            ) -> Check {
                value.serialize(FiniteNumbers)
            }
            fn end(self) -> Check {
                Ok(())
            }
        }
    };
}
check_elements!(SerializeSeq, serialize_element);
check_elements!(SerializeTuple, serialize_element);
check_elements!(SerializeTupleStruct, serialize_field);
check_elements!(SerializeTupleVariant, serialize_field);
check_elements!(SerializeStruct, serialize_field, _key);
check_elements!(SerializeStructVariant, serialize_field, _key);

impl ser::SerializeMap for FiniteNumbers {
    type Ok = ();
    type Error = serde_json::Error;
    fn serialize_key<T: Serialize + ?Sized>(&mut self, value: &T) -> Check {
        value.serialize(FiniteNumbers)
    }
    fn serialize_value<T: Serialize + ?Sized>(&mut self, value: &T) -> Check {
        value.serialize(FiniteNumbers)
    }
    fn end(self) -> Check {
        Ok(())
    }
}
