#ifndef ICON_UTILS_STATUS_HELPERS_H_
#define ICON_UTILS_STATUS_HELPERS_H_

#include "icon/utils/status.h"

namespace intrinsic {

// Returns `new_status` if it is non-OK, `previous_status` otherwise.
//
// Example (Foo() will return the last non-Ok status returned by Bar1(), Bar2()
// or Bar3()):
//
//    RealtimeStatus Bar1();
//    RealtimeStatus Bar2();
//    RealtimeStatus Bar3();
//
//    RealtimeStatus Foo() {
//      RealtimeStatus status = Bar1();
//      status = OverwriteIfError(status, Bar2());
//      status = OverwriteIfError(status, Bar3());
//      return status;
//    }
inline RealtimeStatus OverwriteIfError(const RealtimeStatus& previous_status,
                                       const RealtimeStatus& new_status) {
  if (!new_status.ok()) return new_status;
  return previous_status;
}

// Returns `new_status` if it is non-OK, `previous_status` otherwise.
//
// Example (Foo() will return the last non-Ok status returned by Bar1(), Bar2()
// or Bar3()):
//
//    Status Bar1();
//    Status Bar2();
//    Status Bar3();
//
//    Status Foo() {
//      Status status = Bar1();
//      status = OverwriteIfError(status, Bar2());
//      status = OverwriteIfError(status, Bar3());
//      return status;
//    }
inline Status OverwriteIfError(const Status& previous_status,
                               const Status& new_status) {
  if (!new_status.ok()) return new_status;
  return previous_status;
}

// Returns `new_status` if `previous_status` is OK.
// Used to capture the first non-OK status in a list of sequential calls.
//
// Example (Foo() will return the first non-OK status returned by Bar1(), Bar2()
// or Bar3()):
//    RealtimeStatus Bar1();
//    RealtimeStatus Bar2();
//    RealtimeStatus Bar3();
//
//    RealtimeStatus Foo() {
//      RealtimeStatus status = Bar1();
//      status = OverwriteIfNotInError(status, Bar2());
//      status = OverwriteIfNotInError(status, Bar3());
//      return status;
//    }
inline RealtimeStatus OverwriteIfNotInError(
    const RealtimeStatus& previous_status, const RealtimeStatus& new_status) {
  if (previous_status.ok()) {
    return new_status;
  } else {
    return previous_status;
  }
}

// Returns `new_status` if `previous_status` is OK.
// Used to capture the first non-OK status in a list of sequential calls.
//
// Example (Foo() will return the first non-OK status returned by Bar1(), Bar2()
// or Bar3()):
//    Status Bar1();
//    Status Bar2();
//    Status Bar3();
//
//    Status Foo() {
//      Status status = Bar1();
//      status = OverwriteIfNotInError(status, Bar2());
//      status = OverwriteIfNotInError(status, Bar3());
//      return status;
//    }
inline Status OverwriteIfNotInError(const Status& previous_status,
                                    const Status& new_status) {
  if (previous_status.ok()) {
    return new_status;
  } else {
    return previous_status;
  }
}
}  // namespace intrinsic

#endif  // ICON_UTILS_STATUS_HELPERS_H_
