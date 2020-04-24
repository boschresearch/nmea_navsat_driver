# Software License Agreement (BSD License)
#
# Copyright (c) 2019-2020, Robert Bosch GmbH
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import struct

import rospy


def check_ubx_checksum(sentence):
    """Calculate and compare the checksum of a UBX sentence.

    Args:
        sentence (str): The UBX sentence to check.

    Return True if the calculated checksum of the sentence matches the one provided.
    """
    a = b = 0
    i = 2

    while (i < len(sentence) - 2):
        a = a + struct.unpack('B', sentence[i])[0]
        a = a % 256
        b = b + a
        b = b % 256
        i = i + 1

    return (a, b) == struct.unpack('<BB', sentence[-2:])


def parse_ubx_sentence(ubx_sentence, sentence_definitions):
    """Parse a UBX sentence string into a dictionary.

    Args:
        ubx_sentence (str): A single UBX sentence of one of sentence definitions.
        sentence_definitions (dict): A dictionary containing parsing information for all known UBX sentences.

    Return:
        A dict mapping string field names to values for each field in the UBX sentence or
        False if the sentence could not be parsed.
    """
    # data integrity has already been checked in the interface driver
    # data pre- and suffix around payload
    prefix = 6
    suffix = 2

    sentence_number = struct.unpack('>H', ubx_sentence[2:4])[0]
    sentence_definition = sentence_definitions[sentence_number]

    values = struct.unpack(sentence_definition['format'], ubx_sentence[prefix:prefix + sentence_definition['minlength']])
    sentence = {sentence_definition['name']: dict(zip(sentence_definition['entries'], values))}

    # variable length sentences
    if sentence_definition['minlength'] != sentence_definition['maxlength']:
        repetition_count = sentence[sentence_definition['name']][sentence_definition['repeated-block-counter']]
        repetition_length = sentence_definition['repeated-block-length']
        repetition_format = sentence_definition['repeated-block-format']
        repetition_name = sentence_definition['repeated-block-name']

        if len(ubx_sentence) != prefix + suffix + sentence_definition['minlength'] + repetition_count * repetition_length:
            rospy.logwarn("Discarding invalid length UBX sentence with class {} and id {}"
                          .format(
                            struct.unpack('<B', ubx_sentence[2:3])[0],
                            struct.unpack('<B', ubx_sentence[3:4])[0]
                          ))
            return {}
        else:
            blocks = {}
            for i in range(0, repetition_count):
                block_start = prefix + sentence_definition['minlength'] + i * repetition_length
                blocks[i] = struct.unpack(repetition_format, ubx_sentence[block_start:block_start + repetition_length])[0]

            sentence[sentence_definition['name']][repetition_name] = blocks

    return sentence
