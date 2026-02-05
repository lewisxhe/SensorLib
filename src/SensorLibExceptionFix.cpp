/**
 * @file SensorLibExceptionFix.cpp
 * @brief Exception compatibility layer for Arduino nRF52 platform without C++ exceptions support
 * 
 * This file provides minimal implementations of missing C++ standard library functions
 * required for compiling with -fno-exceptions flag (default in Arduino nRF52 BSP).
 * 
 * @note This file is only compiled when targeting Arduino nRF52 architecture with 
 *       C++ exceptions disabled (Only Arduino IDE). It uses weak symbol attributes to avoid conflicts
 *       with any potential existing implementations.
 */

// Only activate this fix for Arduino nRF52 platforms with C++ exceptions disabled
#if defined(ARDUINO_ARCH_NRF52) && !defined(__cpp_exceptions)

#include <assert.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

namespace std {
    /**
     * @brief Stub implementation of __throw_bad_function_call()
     * 
     * This function is called by std::function when attempting to invoke an empty/null
     * function object. In a proper C++ environment, this would throw std::bad_function_call.
     * Since exceptions are disabled, we provide a stub that asserts.
     * 
     * @note Marked as weak symbol to allow overriding by other libraries or user code
     */
    __attribute__((weak))
    void __throw_bad_function_call() {
        assert(false && "Attempted to call an empty std::function");
    }
    
    /**
     * @brief Stub implementation of __throw_length_error()
     * 
     * Called by standard containers when a length error would normally occur
     * (e.g., vector::reserve with size exceeding max_size()).
     * 
     * @param msg Error message (included in assert output for debugging)
     */
    __attribute__((weak))
    void __throw_length_error(char const* msg) {
        // Use assert to indicate programming error
        assert(false && msg);
    }
    
    /**
     * @brief Stub implementation of __throw_out_of_range()
     * 
     * Called by standard containers when an out-of-bounds access would normally occur
     * (e.g., vector::at() with invalid index).
     * 
     * @param msg Error message (included in assert output for debugging)
     */
    __attribute__((weak))
    void __throw_out_of_range(char const* msg) {
        assert(false && msg);
    }
}

#pragma GCC diagnostic pop

#endif // defined(ARDUINO_ARCH_NRF52) && !defined(__cpp_exceptions)