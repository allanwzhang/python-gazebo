// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vector8d.proto

#ifndef PROTOBUF_vector8d_2eproto__INCLUDED
#define PROTOBUF_vector8d_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3000000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3000000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace tarotPB {
namespace msgs {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_vector8d_2eproto();
void protobuf_AssignDesc_vector8d_2eproto();
void protobuf_ShutdownFile_vector8d_2eproto();

class Vector8d;

// ===================================================================

class Vector8d : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:tarotPB.msgs.Vector8d) */ {
 public:
  Vector8d();
  virtual ~Vector8d();

  Vector8d(const Vector8d& from);

  inline Vector8d& operator=(const Vector8d& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Vector8d& default_instance();

  void Swap(Vector8d* other);

  // implements Message ----------------------------------------------

  inline Vector8d* New() const { return New(NULL); }

  Vector8d* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Vector8d& from);
  void MergeFrom(const Vector8d& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(Vector8d* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required double motor_1 = 1;
  bool has_motor_1() const;
  void clear_motor_1();
  static const int kMotor1FieldNumber = 1;
  double motor_1() const;
  void set_motor_1(double value);

  // required double motor_2 = 2;
  bool has_motor_2() const;
  void clear_motor_2();
  static const int kMotor2FieldNumber = 2;
  double motor_2() const;
  void set_motor_2(double value);

  // required double motor_3 = 3;
  bool has_motor_3() const;
  void clear_motor_3();
  static const int kMotor3FieldNumber = 3;
  double motor_3() const;
  void set_motor_3(double value);

  // required double motor_4 = 4;
  bool has_motor_4() const;
  void clear_motor_4();
  static const int kMotor4FieldNumber = 4;
  double motor_4() const;
  void set_motor_4(double value);

  // required double motor_5 = 5;
  bool has_motor_5() const;
  void clear_motor_5();
  static const int kMotor5FieldNumber = 5;
  double motor_5() const;
  void set_motor_5(double value);

  // required double motor_6 = 6;
  bool has_motor_6() const;
  void clear_motor_6();
  static const int kMotor6FieldNumber = 6;
  double motor_6() const;
  void set_motor_6(double value);

  // required double motor_7 = 7;
  bool has_motor_7() const;
  void clear_motor_7();
  static const int kMotor7FieldNumber = 7;
  double motor_7() const;
  void set_motor_7(double value);

  // required double motor_8 = 8;
  bool has_motor_8() const;
  void clear_motor_8();
  static const int kMotor8FieldNumber = 8;
  double motor_8() const;
  void set_motor_8(double value);

  // @@protoc_insertion_point(class_scope:tarotPB.msgs.Vector8d)
 private:
  inline void set_has_motor_1();
  inline void clear_has_motor_1();
  inline void set_has_motor_2();
  inline void clear_has_motor_2();
  inline void set_has_motor_3();
  inline void clear_has_motor_3();
  inline void set_has_motor_4();
  inline void clear_has_motor_4();
  inline void set_has_motor_5();
  inline void clear_has_motor_5();
  inline void set_has_motor_6();
  inline void clear_has_motor_6();
  inline void set_has_motor_7();
  inline void clear_has_motor_7();
  inline void set_has_motor_8();
  inline void clear_has_motor_8();

  // helper for ByteSize()
  int RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double motor_1_;
  double motor_2_;
  double motor_3_;
  double motor_4_;
  double motor_5_;
  double motor_6_;
  double motor_7_;
  double motor_8_;
  friend void  protobuf_AddDesc_vector8d_2eproto();
  friend void protobuf_AssignDesc_vector8d_2eproto();
  friend void protobuf_ShutdownFile_vector8d_2eproto();

  void InitAsDefaultInstance();
  static Vector8d* default_instance_;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Vector8d

// required double motor_1 = 1;
inline bool Vector8d::has_motor_1() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Vector8d::set_has_motor_1() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Vector8d::clear_has_motor_1() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Vector8d::clear_motor_1() {
  motor_1_ = 0;
  clear_has_motor_1();
}
inline double Vector8d::motor_1() const {
  // @@protoc_insertion_point(field_get:tarotPB.msgs.Vector8d.motor_1)
  return motor_1_;
}
inline void Vector8d::set_motor_1(double value) {
  set_has_motor_1();
  motor_1_ = value;
  // @@protoc_insertion_point(field_set:tarotPB.msgs.Vector8d.motor_1)
}

// required double motor_2 = 2;
inline bool Vector8d::has_motor_2() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Vector8d::set_has_motor_2() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Vector8d::clear_has_motor_2() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Vector8d::clear_motor_2() {
  motor_2_ = 0;
  clear_has_motor_2();
}
inline double Vector8d::motor_2() const {
  // @@protoc_insertion_point(field_get:tarotPB.msgs.Vector8d.motor_2)
  return motor_2_;
}
inline void Vector8d::set_motor_2(double value) {
  set_has_motor_2();
  motor_2_ = value;
  // @@protoc_insertion_point(field_set:tarotPB.msgs.Vector8d.motor_2)
}

// required double motor_3 = 3;
inline bool Vector8d::has_motor_3() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Vector8d::set_has_motor_3() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Vector8d::clear_has_motor_3() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Vector8d::clear_motor_3() {
  motor_3_ = 0;
  clear_has_motor_3();
}
inline double Vector8d::motor_3() const {
  // @@protoc_insertion_point(field_get:tarotPB.msgs.Vector8d.motor_3)
  return motor_3_;
}
inline void Vector8d::set_motor_3(double value) {
  set_has_motor_3();
  motor_3_ = value;
  // @@protoc_insertion_point(field_set:tarotPB.msgs.Vector8d.motor_3)
}

// required double motor_4 = 4;
inline bool Vector8d::has_motor_4() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Vector8d::set_has_motor_4() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Vector8d::clear_has_motor_4() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Vector8d::clear_motor_4() {
  motor_4_ = 0;
  clear_has_motor_4();
}
inline double Vector8d::motor_4() const {
  // @@protoc_insertion_point(field_get:tarotPB.msgs.Vector8d.motor_4)
  return motor_4_;
}
inline void Vector8d::set_motor_4(double value) {
  set_has_motor_4();
  motor_4_ = value;
  // @@protoc_insertion_point(field_set:tarotPB.msgs.Vector8d.motor_4)
}

// required double motor_5 = 5;
inline bool Vector8d::has_motor_5() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Vector8d::set_has_motor_5() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Vector8d::clear_has_motor_5() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Vector8d::clear_motor_5() {
  motor_5_ = 0;
  clear_has_motor_5();
}
inline double Vector8d::motor_5() const {
  // @@protoc_insertion_point(field_get:tarotPB.msgs.Vector8d.motor_5)
  return motor_5_;
}
inline void Vector8d::set_motor_5(double value) {
  set_has_motor_5();
  motor_5_ = value;
  // @@protoc_insertion_point(field_set:tarotPB.msgs.Vector8d.motor_5)
}

// required double motor_6 = 6;
inline bool Vector8d::has_motor_6() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Vector8d::set_has_motor_6() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Vector8d::clear_has_motor_6() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Vector8d::clear_motor_6() {
  motor_6_ = 0;
  clear_has_motor_6();
}
inline double Vector8d::motor_6() const {
  // @@protoc_insertion_point(field_get:tarotPB.msgs.Vector8d.motor_6)
  return motor_6_;
}
inline void Vector8d::set_motor_6(double value) {
  set_has_motor_6();
  motor_6_ = value;
  // @@protoc_insertion_point(field_set:tarotPB.msgs.Vector8d.motor_6)
}

// required double motor_7 = 7;
inline bool Vector8d::has_motor_7() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Vector8d::set_has_motor_7() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Vector8d::clear_has_motor_7() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Vector8d::clear_motor_7() {
  motor_7_ = 0;
  clear_has_motor_7();
}
inline double Vector8d::motor_7() const {
  // @@protoc_insertion_point(field_get:tarotPB.msgs.Vector8d.motor_7)
  return motor_7_;
}
inline void Vector8d::set_motor_7(double value) {
  set_has_motor_7();
  motor_7_ = value;
  // @@protoc_insertion_point(field_set:tarotPB.msgs.Vector8d.motor_7)
}

// required double motor_8 = 8;
inline bool Vector8d::has_motor_8() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Vector8d::set_has_motor_8() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Vector8d::clear_has_motor_8() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Vector8d::clear_motor_8() {
  motor_8_ = 0;
  clear_has_motor_8();
}
inline double Vector8d::motor_8() const {
  // @@protoc_insertion_point(field_get:tarotPB.msgs.Vector8d.motor_8)
  return motor_8_;
}
inline void Vector8d::set_motor_8(double value) {
  set_has_motor_8();
  motor_8_ = value;
  // @@protoc_insertion_point(field_set:tarotPB.msgs.Vector8d.motor_8)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace tarotPB

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_vector8d_2eproto__INCLUDED