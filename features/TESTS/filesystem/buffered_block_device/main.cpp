/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "greentea-client/test_env.h"
#include "unity.h"
#include "utest.h"

#include "BufferedBlockDevice.h"
#include "HeapBlockDevice.h"
#include <stdlib.h>

using namespace utest::v1;

static const bd_size_t heap_erase_size = 512;
static const bd_size_t heap_prog_size = heap_erase_size;
static const bd_size_t heap_read_size = 256;
static const bd_size_t num_blocks = 4;

void functionality_test()
{
    uint8_t *dummy = new (std::nothrow) uint8_t[num_blocks * heap_erase_size + heap_prog_size];
    TEST_SKIP_UNLESS_MESSAGE(dummy, "Not enough memory for test");
    delete[] dummy;

    HeapBlockDevice heap_bd(num_blocks * heap_erase_size, heap_read_size, heap_prog_size, heap_erase_size);
    BufferedBlockDevice bd(&heap_bd);

    int err = bd.init();
    TEST_ASSERT_EQUAL(0, err);

    uint8_t *read_buf, *write_buf;
    read_buf = new (std::nothrow) uint8_t[heap_prog_size];
    TEST_SKIP_UNLESS_MESSAGE(read_buf, "Not enough memory for test");
    write_buf = new (std::nothrow) uint8_t[heap_prog_size];
    TEST_SKIP_UNLESS_MESSAGE(write_buf, "Not enough memory for test");

    TEST_ASSERT_EQUAL(1, bd.get_read_size());
    TEST_ASSERT_EQUAL(1, bd.get_program_size());
    TEST_ASSERT_EQUAL(heap_erase_size, bd.get_erase_size());

    for (bd_size_t i = 0; i < num_blocks; i++) {
        memset(write_buf, i, heap_prog_size);
        err = heap_bd.program(write_buf, i * heap_prog_size, heap_prog_size);
        TEST_ASSERT_EQUAL(0, err);
    }

    err = bd.read(read_buf, heap_prog_size + heap_prog_size / 2, 1);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL(1, read_buf[0]);

    err = bd.read(read_buf, 2 * heap_prog_size + heap_prog_size / 2, 4);
    TEST_ASSERT_EQUAL(0, err);
    memset(write_buf, 2, 4);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, 4);

    memset(write_buf, 1, heap_prog_size);
    memset(write_buf + 64, 0x5A, 8);
    memset(write_buf + 72, 0xA5, 8);
    err = bd.program(write_buf + 64, heap_prog_size + 64, 8);
    TEST_ASSERT_EQUAL(0, err);
    err = bd.program(write_buf + 72, heap_prog_size + 72, 8);
    TEST_ASSERT_EQUAL(0, err);
    err = bd.read(read_buf, heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);
    memset(write_buf, 1, heap_prog_size);
    // Underlying BD should not be updated before sync
    err = heap_bd.read(read_buf, heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);
    err = bd.sync();
    TEST_ASSERT_EQUAL(0, err);
    memset(write_buf + 64, 0x5A, 8);
    memset(write_buf + 72, 0xA5, 8);
    // Should be updated now
    err = bd.read(read_buf, heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);
    err = heap_bd.read(read_buf, heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);

    memset(write_buf, 0xAA, 16);
    memset(write_buf + 16, 0xBB, 16);
    err = bd.program(write_buf, 3 * heap_prog_size - 16, 32);
    TEST_ASSERT_EQUAL(0, err);
    // First block should sync, but second still shouldn't
    memset(write_buf, 2, heap_prog_size - 16);
    memset(write_buf + heap_prog_size - 16, 0xAA, 16);
    err = heap_bd.read(read_buf, 2 * heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);
    memset(write_buf, 3, heap_prog_size);
    err = heap_bd.read(read_buf, 3 * heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);
    memset(write_buf, 0xBB, 16);
    err = bd.read(read_buf, 3 * heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);
    // Moving to another block should automatically sync
    err = bd.read(read_buf, 15, 1);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL(0, read_buf[0]);
    err = heap_bd.read(read_buf, 3 * heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);
    err = bd.read(read_buf, 3 * heap_prog_size, heap_prog_size);
    TEST_ASSERT_EQUAL(0, err);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(write_buf, read_buf, heap_prog_size);

    delete[] read_buf;
    delete[] write_buf;
}

// Test setup
utest::v1::status_t test_setup(const size_t number_of_cases)
{
    GREENTEA_SETUP(30, "default_auto");
    return verbose_test_setup_handler(number_of_cases);
}

Case cases[] = {
    Case("BufferedBlockDevice functionality test", functionality_test),
};

Specification specification(test_setup, cases);

int main()
{
    return !Harness::run(specification);
}

