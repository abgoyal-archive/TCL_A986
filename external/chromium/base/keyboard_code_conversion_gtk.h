// Copyright (c) 2009 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.


// WindowsKeyCodeForGdkKeyCode is copied from platform/gtk/KeyEventGtk.cpp

#ifndef BASE_KEYBOARD_CODE_CONVERSION_GTK_H_
#define BASE_KEYBOARD_CODE_CONVERSION_GTK_H_

#include "base/keyboard_codes_posix.h"

namespace base {

base::KeyboardCode WindowsKeyCodeForGdkKeyCode(int keycode);

int GdkKeyCodeForWindowsKeyCode(base::KeyboardCode keycode, bool shift);

} // namespace

#endif  // BASE_KEYBOARD_CODE_CONVERSION_GTK_H_
