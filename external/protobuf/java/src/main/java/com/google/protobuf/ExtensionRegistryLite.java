// Protocol Buffers - Google's data interchange format
// Copyright 2008 Google Inc.  All rights reserved.
// http://code.google.com/p/protobuf/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

package com.google.protobuf;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class ExtensionRegistryLite {
  /** Construct a new, empty instance. */
  public static ExtensionRegistryLite newInstance() {
    return new ExtensionRegistryLite();
  }

  /** Get the unmodifiable singleton empty instance. */
  public static ExtensionRegistryLite getEmptyRegistry() {
    return EMPTY;
  }

  /** Returns an unmodifiable view of the registry. */
  public ExtensionRegistryLite getUnmodifiable() {
    return new ExtensionRegistryLite(this);
  }

  /**
   * Find an extension by containing type and field number.
   *
   * @return Information about the extension if found, or {@code null}
   *         otherwise.
   */
  @SuppressWarnings("unchecked")
  public <ContainingType extends MessageLite>
      GeneratedMessageLite.GeneratedExtension<ContainingType, ?>
        findLiteExtensionByNumber(
          final ContainingType containingTypeDefaultInstance,
          final int fieldNumber) {
    return (GeneratedMessageLite.GeneratedExtension<ContainingType, ?>)
      extensionsByNumber.get(
        new ObjectIntPair(containingTypeDefaultInstance, fieldNumber));
  }

  /** Add an extension from a lite generated file to the registry. */
  public final void add(
      final GeneratedMessageLite.GeneratedExtension<?, ?> extension) {
    extensionsByNumber.put(
      new ObjectIntPair(extension.getContainingTypeDefaultInstance(),
                        extension.getNumber()),
      extension);
  }

  // =================================================================
  // Private stuff.

  // Constructors are package-private so that ExtensionRegistry can subclass
  // this.

  ExtensionRegistryLite() {
    this.extensionsByNumber =
        new HashMap<ObjectIntPair,
                    GeneratedMessageLite.GeneratedExtension<?, ?>>();
  }

  ExtensionRegistryLite(ExtensionRegistryLite other) {
    if (other == EMPTY) {
      this.extensionsByNumber = Collections.emptyMap();
    } else {
      this.extensionsByNumber =
        Collections.unmodifiableMap(other.extensionsByNumber);
    }
  }

  private final Map<ObjectIntPair,
                    GeneratedMessageLite.GeneratedExtension<?, ?>>
      extensionsByNumber;

  private ExtensionRegistryLite(boolean empty) {
    this.extensionsByNumber = Collections.emptyMap();
  }
  private static final ExtensionRegistryLite EMPTY =
    new ExtensionRegistryLite(true);

  /** A (Object, int) pair, used as a map key. */
  private static final class ObjectIntPair {
    private final Object object;
    private final int number;

    ObjectIntPair(final Object object, final int number) {
      this.object = object;
      this.number = number;
    }

    @Override
    public int hashCode() {
      return System.identityHashCode(object) * ((1 << 16) - 1) + number;
    }
    @Override
    public boolean equals(final Object obj) {
      if (!(obj instanceof ObjectIntPair)) {
        return false;
      }
      final ObjectIntPair other = (ObjectIntPair)obj;
      return object == other.object && number == other.number;
    }
  }
}
