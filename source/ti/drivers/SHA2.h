/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       SHA2.h
 *
 *  @brief      SHA2 driver header
 *
 * @warning     This is a beta API. It may change in future releases.
 *
 *  # Overview #
 *  SHA2 (Secure Hash Algorithm 2) is a cryptographic hashing algorithm that
 *  maps an input of arbitrary length to a fixed-length output with negligible
 *  probability of collision. A collision would occur when two different inputs
 *  map to the same output.
 *
 *  It is not currently technologicaly feasible to derive an input from
 *  the hash digest (output) itself.
 *
 *  Hashes are often used to ensure the integrity of messages. They are also
 *  used to as constituent parts of more complicated cyptographic schemes.
 *  HMAC is a message authentication code that is based on hash functions such
 *  as SHA2 rather than a block cipher.
 *  Hashes may themselves be used as or form a part of key derivation functions
 *  used to derive symmetric keys from sources of entropy such as an Elliptic
 *  Curve Diffie-Helman key exchange (ECDH).
 *
 *  SHA2 is not actually a single algorithms but a suite of similar algorithms
 *  that produce hash digests of different lengths. 224, 256, 384, and 512-bit
 *  outputs are available.
 *
 *  "Hash" may refer to either the process of hashing when used as a verb and
 *  the output digest when used as a noun.
 *
 *  # Usage #
 *
 *  ## Before starting a SHA2 operation #
 *
 *  Before starting a SHA2 operation, the application must do the following:
 *      - Call SHA2_init() to initialize the driver
 *      - Call SHA2_Params_init() to initialize the SHA2_Params to default values.
 *      - Modify the SHA2_Params as desired
 *      - Call SHA2_open() to open an instance of the driver
 *      - Initialise one of the SHA2_Operation structs using the corresponding
 *        initialization function.
 *
 *  ## Starting a SHA2 operation #
 *
 *  There are two general ways to execute a SHA2 operation. In one call or multiple.
 *
 *  The SHA2_oneStepHash function performs a SHA2 operation in a single call.
 *  It will always be the most highly optimized routine with the least overhead and
 *  the fastest runtime. However, it requires that the entire input message be
 *  available to the function in a contiguous location at the start of the call.
 *  When trying to operate on data that is too large to fit into available memory,
 *  partial processing is more advisable. The single call operation is required
 *  when hashing a message with a lenth smaller than or equal to one hash-block length.
 *  All devices support single call operations.
 *
 *  ## After the SHA2 operation completes #
 *
 *  After the SHA2 operation completes, the application should either start another operation
 *  or close the driver by calling SHA2_close().
 *
 *  ## SHA2 Driver Configuration #
 *
 *  In order to use the SHA2 APIs, the application is required
 *  to provide device-specific SHA2 configuration in the Board.c file.
 *  The SHA2 driver interface defines a configuration data structure:
 *
 *  @code
 *  typedef struct SHA2_Config_ {
 *      void                   *object;
 *      void          const    *hwAttrs;
 *  } SHA2_Config;
 *  @endcode
 *
 *  The application must declare an array of SHA2_Config elements, named
 *  SHA2_config[].  Each element of SHA2_config[] must be populated with
 *  pointers to a device specific SHA2 driver implementation's driver object and
 *  hardware attributes.  The hardware attributes define properties such
 *  as the SHA2 peripheral's base address.
 *  Each element in SHA2_config[] corresponds to an SHA2 instance
 *  and none of the elements should have NULL pointers.
 *  There is no correlation between the index and the
 *  peripheral designation (such as SHA20 or SHA21).  For example, it is
 *  possible to use SHA2_config[0] for SHA21. Multiple drivers and driver
 *  instances may all access the same underlying hardware. This is transparent
 *  to the application. Mutual exclusion is performed automatically by the
 *  drivers as necessary.
 *
 *  Because the SHA2 configuration is very device dependent, you will need to
 *  check the doxygen for the device specific SHA2 implementation.  There, you
 *  will find a description of the SHA2 hardware attributes.  Please also
 *  refer to the Board.c file of any of your examples to see the SHA2
 *  configuration.
 *
 *  ## SHA2 Parameters #
 *
 *  The #SHA2_Params structure is passed to the SHA2_open() call.  If NULL
 *  is passed for the parameters, SHA2_open() uses default parameters.
 *  A #SHA2_Params structure is initialized with default values by passing
 *  it to SHA2_Params_init().
 *  Some of the SHA2 parameters are described below.  To see brief descriptions
 *  of all the parameters, see #SHA2_Params.
 *
 *  ## Examples #
 *
 *  ### Single call SHA2 256-bit hash in blocking mode #
 *  @code
 *
 *  #include <ti/drivers/SHA2.h>
 *
 *  SHA2_Handle handle;
 *  HA2_OperationOneStepHash operationOneStepHash;
 *
 *  uint8_t message[] = ""; // The message is the empty string
 *  uint8_t digest[SHA2_DIGEST_LENGTH_BYTES_256];
 *
 *  handle = SHA2_open(0, NULL);
 *  if (!handle) {
 *      while(1);
 *  }
 *
 *  SHA2_OperationOneStepHash_init(&operationOneStepHash);
 *  operationOneStepHash.hashSize       = SHA2_HASH_SIZE_256;
 *  operationOneStepHash.message        = message;
 *  operationOneStepHash.digest         = digest;
 *  operationOneStepHash.totalLength    = 0;
 *
 *  int_fast16_t testResult = SHA2_oneStepHash(handle, &operationOneStepHash);
 *
 *  if (testResult != SHA2_STATUS_SUCCESS) {
 *      // handle error
 *      while(1);
 *  }
 *
 *  // The resultant digest should be:
 *  // 0xE3, 0xB0, 0xC4, 0x42,
 *  // 0x98, 0xFC, 0x1C, 0x14,
 *  // 0x9A, 0xFB, 0xF4, 0xC8,
 *  // 0x99, 0x6F, 0xB9, 0x24,
 *  // 0x27, 0xAE, 0x41, 0xE4,
 *  // 0x64, 0x9B, 0x93, 0x4C,
 *  // 0xA4, 0x95, 0x99, 0x1B,
 *  // 0x78, 0x52, 0xB8, 0x55,
 *
 *  SHA2_close(handle);
 *
 *  @endcode
 *
 *  ### Partial hash with exported intermediate context #
 *  @code
 *
 *  #include <ti/drivers/SHA2.h>
 *
 *  SHA2_Handle handle;
 *  HA2_OperationStartHash operationStartHash;
 *  HA2_OperationFinishHash operationFinishHash;
 *
 *   // 112-byte string
 *  uint8_t message[] = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmgh
 *                       ijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnop
 *                       qrstnopqrstu";
 *  uint8_t digest[SHA2_DIGEST_LENGTH_BYTES_256];
 *
 *  handle = SHA2_open(0, NULL);
 *  if (!handle) {
 *      while(1);
 *  }
 *
 *  SHA2_OperationStartHash_init(&operationStartHash);
 *  operationStartHash.hashSize              = SHA2_HASH_SIZE_256;
 *  operationStartHash.length                = SHA2_BLOCK_SIZE_BYTES_256;
 *  operationStartHash.message               = message;
 *  operationStartHash.intermediateDigest    = digest;
 *
 *  int_fast16_t testResult = SHA2_startHash(handle, &operationStartHash);
 *
 *  if (testResult != SHA2_STATUS_SUCCESS) {
 *      while(1);
 *  }
 *
 *  SHA2_OperationFinishHash_init(&operationFinishHash);
 *  operationFinishHash.hashSize             = SHA2_HASH_SIZE_256;
 *  operationFinishHash.segmentLength        = 112 - SHA2_BLOCK_SIZE_BYTES_256;
 *  operationFinishHash.message              = message + SHA2_BLOCK_SIZE_BYTES_256;
 *  operationFinishHash.intermediateDigest   = digest;
 *  operationFinishHash.finalDigest          = digest;
 *  operationFinishHash.totalLength          = 112;
 *
 *  testResult = SHA2_finishHash(handle,  &operationFinishHash);
 *
 *  if (testResult != SHA2_STATUS_SUCCESS) {
 *      while(1);
 *  }
 *
 *  // The resultant digest should be:
 *  // 0xcf, 0x5b, 0x16, 0xa7,
 *  // 0x78, 0xaf, 0x83, 0x80,
 *  // 0x03, 0x6c, 0xe5, 0x9e,
 *  // 0x7b, 0x04, 0x92, 0x37,
 *  // 0x0b, 0x24, 0x9b, 0x11,
 *  // 0xe8, 0xf0, 0x7a, 0x51,
 *  // 0xaf, 0xac, 0x45, 0x03,
 *  // 0x7a, 0xfe, 0xe9, 0xd1
 *
 *  SHA2_close(handle);
 *
 *  @endcode
 *
 */

#ifndef ti_drivers_SHA2__include
#define ti_drivers_SHA2__include

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/drivers/cryptoutils/cryptokey/CryptoKey.h>

/**
 *  @defgroup SHA2_CONTROL SHA2_control command and status codes
 *  These SHA2 macros are reservations for SHA2.h
 *  @{
 */

/*!
 * Common SHA2_control command code reservation offset.
 * SHA2 driver implementations should offset command codes with SHA2_CMD_RESERVED
 * growing positively
 *
 * Example implementation specific command codes:
 * @code
 * #define SHA2XYZ_CMD_COMMAND0     SHA2_CMD_RESERVED + 0
 * #define SHA2XYZ_CMD_COMMAND1     SHA2_CMD_RESERVED + 1
 * @endcode
 */
#define SHA2_CMD_RESERVED           (32)

/*!
 * Common SHA2_control status code reservation offset.
 * SHA2 driver implementations should offset status codes with
 * SHA2_STATUS_RESERVED growing negatively.
 *
 * Example implementation specific status codes:
 * @code
 * #define SHA2XYZ_STATUS_ERROR0    SHA2_STATUS_RESERVED - 0
 * #define SHA2XYZ_STATUS_ERROR1    SHA2_STATUS_RESERVED - 1
 * #define SHA2XYZ_STATUS_ERROR2    SHA2_STATUS_RESERVED - 2
 * @endcode
 */
#define SHA2_STATUS_RESERVED        (-32)

/**
 *  @defgroup SHA2_STATUS Status Codes
 *  SHA2_STATUS_* macros are general status codes returned by SHA2 functions
 *  @{
 *  @ingroup SHA2_CONTROL
 */

/*!
 * @brief   Successful status code.
 *
 * Functions return SHA2_STATUS_SUCCESS if the function was executed
 * successfully.
 */
#define SHA2_STATUS_SUCCESS         (0)

/*!
 * @brief   Generic error status code.
 *
 * Functions return SHA2_STATUS_ERROR if the function was not executed
 * successfully and no more specific error is applicable.
 */
#define SHA2_STATUS_ERROR           (-1)

/*!
 * @brief   An error status code returned by SHA2_control() for undefined
 * command codes.
 *
 * SHA2_control() returns SHA2_STATUS_UNDEFINEDCMD if the control code is not
 * recognized by the driver implementation.
 */
#define SHA2_STATUS_UNDEFINEDCMD    (-2)

/*!
 * @brief   An error status code returned if the hardware or software resource
 * is currently unavailable.
 *
 * SHA2 driver implementations may have hardware or software limitations on how
 * many clients can simultaneously perform operations. This status code is returned
 * if the mutual exclusion mechanism signals that an operation cannot currently be performed.
 */
#define SHA2_STATUS_RESOURCE_UNAVAILABLE (-3)


/** @}*/

/**
 *  @defgroup SHA2_CMD Command Codes
 *  SHA2_CMD_* macros are general command codes for SHA2_control(). Not all SHA2
 *  driver implementations support these command codes.
 *  @{
 *  @ingroup SHA2_CONTROL
 */

/* Add SHA2_CMD_<commands> here */

/** @}*/

/** @}*/

/*!
 *  @brief  A handle that is returned from an SHA2_open() call.
 */
typedef struct SHA2_Config_    *SHA2_Handle;

/*!
 * @brief   The way in which SHA2 function calls return after performing an
 * operation.
 *
 * Not all SHA2 operations exhibit the specified return behavor. Functions that do not
 * require significant computation and cannot offload that computation to a background thread
 * behave like regular functions. Which functions exhibit the specfied return behavior is not
 * implementation dependent. Specifically, a software-backed implementation run on the same
 * CPU as the application will emulate the return behavior while not actually offloading
 * the computation to the background thread.
 *
 * SHA2 functions exhibiting the specified return behavior have restrictions on the
 * context from which they may be called.
 *
 * |                                | Task  | Hwi   | Swi   |
 * |--------------------------------|-------|-------|-------|
 * |SHA2_RETURN_BEHAVIOR_CALLBACK   | X     | X     | X     |
 * |SHA2_RETURN_BEHAVIOR_BLOCKING   | X     |       |       |
 * |SHA2_RETURN_BEHAVIOR_POLLING    | X     | X     | X     |
 *
 */
typedef enum SHA2_ReturnBehavior_ {
    SHA2_RETURN_BEHAVIOR_CALLBACK = 1,      /*!< The function call will return immediately while the
                                             *   SHA2 operation goes on in the background. The registered
                                             *   callback function is called after the operation completes.
                                             *   The context the callback function is called (task, HWI, SWI)
                                             *   is implementation-dependent.
                                             */
    SHA2_RETURN_BEHAVIOR_BLOCKING = 2,      /*!< The function call will block while the SHA2 operation goes
                                             *   on in the background. SHA2 operation results are available
                                             *   after the function returns.
                                             */
    SHA2_RETURN_BEHAVIOR_POLLING  = 4,      /*!< The function call will continuously poll a flag while the SHA2
                                             *   operation goes on in the background. SHA2 operation results
                                             *   are available after the function returns.
                                             */
} SHA2_ReturnBehavior;

/*!
 *  @brief  Enum for the hash digest sizes supported by the driver.
 */
typedef enum SHA2_HashSize_ {
    SHA2_HASH_SIZE_224 = 0,
    SHA2_HASH_SIZE_256 = 1,
    SHA2_HASH_SIZE_384 = 2,
    SHA2_HASH_SIZE_512 = 3,
} SHA2_HashSize;

/*!
 *  @brief  Enum for the hash digest lengths in bytes supported by the driver.
 */
typedef enum SHA2_DigestLengthBytes_ {
    SHA2_DIGEST_LENGTH_BYTES_224 = 28,
    SHA2_DIGEST_LENGTH_BYTES_256 = 32,
    SHA2_DIGEST_LENGTH_BYTES_384 = 48,
    SHA2_DIGEST_LENGTH_BYTES_512 = 64,
} SHA2_DigestLengthBytes;

/*!
 *  @brief  Enum for the block sizes of the algorithms.
 *
 *  SHA2 iteratively consumes segments of the block
 *  size and computes intermediate digests which are
 *  fed back into the algorithm together with the next
 *  segment to compute the next intermediate or final
 *  digest.
 *  The block sizes of the algorithms differ from their
 *  digest lengths. When performing partial hashes,
 *  the segment lengths for all but the last segment
 *  must be multiples of the relevant block size.
 */
typedef enum SHA2_BlockSizeBytes_ {
    SHA2_BLOCK_SIZE_BYTES_224 = 64,
    SHA2_BLOCK_SIZE_BYTES_256 = 64,
    SHA2_BLOCK_SIZE_BYTES_384 = 128,
    SHA2_BLOCK_SIZE_BYTES_512 = 128,
} SHA2_BlockSizeBytes;

/*!
 *  @brief  Struct containing the parameters required to hash
 *          a message in one go.
 */
typedef struct SHA2_OperationOneStep_ {
   SHA2_HashSize        hashSize;       /*!< The hash size to use. This also determines the required size of
                                         *   \c digest.
                                         */
   size_t               totalLength;    /*!< The length of the message to hash. */
   const uint8_t        *message;       /*!< Pointer to the message to hash */
   uint8_t              *digest;        /*!< Pointer to location output digest will be placed.
                                         *   User must allocate enough space for hash result
                                         *   (ex. 32 bytes for 256, 28 bytes for 224).
                                         */
} SHA2_OperationOneStepHash;

/*!
 *  @brief  Struct containing the parameters required to start
 *          hashing a message.
 */
typedef struct SHA2_OperationStartHash_ {
   SHA2_HashSize        hashSize;               /*!< The hash size to use. */
   size_t               length;                 /*!< The length of the message segment to hash, in bytes.  This
                                                 *   length must be a multiple of the hash block size.
                                                 *   The block size for 224 and 256 is 64 bytes and the block
                                                 *   size for 384 and 512 is 128 bytes.
                                                 */
   const uint8_t        *message;               /*!< Pointer to the message segment to hash. */
   uint8_t              *intermediateDigest;    /*!< Pointer to location the intermediate output digest will be placed.
                                                 *   If NULL, the driver will store the intermediate digest itself.
                                                 *   This context may be overwritten by other clients performing their
                                                 *   own hash operations however. The context may also be lost if
                                                 *   going into low power modes in between hash operations.
                                                 *
                                                 *   User must allocate enough space for hash result
                                                 *   (ex. 32 bytes for 256, 28 bytes for 224).
                                                 */
} SHA2_OperationStartHash;



/*!
 *  @brief  Struct containing the parameters required to continue
 *          hashing a message.
 */
typedef struct SHA2_OperationProcessHash_ {
   SHA2_HashSize        hashSize;               /*!< The hash size to use. */
   size_t               length;                 /*!< The length of the message segment to hash, in bytes.  This
                                                 *   length must be a multiple of the hash block size.
                                                 *   The block size for 224 and 256 is 64 bytes and the block
                                                 *   size for 384 and 512 is 128 bytes.
                                                 */
   const uint8_t        *message;               /*!< Pointer to the message segment to hash. */
   uint8_t              *intermediateDigest;    /*!< Pointer to location the previous intermediate output digest
                                                 *   will be loaded from and the newly computed intermediate output
                                                 *   will be written back.
                                                 *
                                                 *   If NULL, the driver will store the intermediate digest itself and
                                                 *   load the previous intermediate digest from within itself.
                                                 *   This context may be overwritten by other clients performing their
                                                 *   own hash operations however. The context may also be lost if
                                                 *   going into low power modes in between hash operations.
                                                 *
                                                 *   User must allocate enough space for hash result
                                                 *   (ex. 32 bytes for 256, 28 bytes for 224).
                                                 */
} SHA2_OperationProcessHash;

/*!
 *  @brief  Struct containing the parameters required to process
 *          the last blocks of a message and finalize the hash.
 */
typedef struct SHA2_OperationFinishlHash_ {
   SHA2_HashSize        hashSize;               /*!< The hash size to use. */
   size_t               segmentLength;          /*!< The length of the message segment to hash, in bytes.
                                                 *   It is not required to be a multiple of the block size.
                                                 */
   size_t               totalLength;            /*!< The total length of the entire message, in bytes.
                                                 *   It is not required to be a multiple of the block size.
                                                 *   This is required for finalization of the hash.
                                                 */
   const uint8_t        *message;               /*!< Pointer to the message segment to hash. */
   const uint8_t        *intermediateDigest;    /*!< Pointer to location the previous intermediate output digest
                                                 *   will be loaded from.
                                                 *
                                                 *   If NULL, the driver will load the previous intermediate digest
                                                 *   from within itself.
                                                 *   This context may have been overwritten by other clients
                                                 *   performing their own hash operations. The context
                                                 *   may also have been be lost after going into low power modes
                                                 *   in between hash operations.
                                                 *
                                                 *   User must allocate enough space for hash result
                                                 *   (ex. 32 bytes for 256, 28 bytes for 224).
                                                 */
   uint8_t              *finalDigest;           /*!< Pointer to the location of the result digest buffer.
                                                 *   It may point to the same location as \c intermediateDigest.
                                                 *   In that case, \c intermediateDigest will be overwritten
                                                 *   with the result digest.
                                                 */
} SHA2_OperationFinishHash;

/*!
 *  @brief  Union containing pointers to all supported operation structs.
 */
typedef union SHA2_Operation_ {
    SHA2_OperationStartHash         *startHash;     /*!< A pointer to a SHA2_OperationStartHash struct */
    SHA2_OperationProcessHash       *processHash;   /*!< A pointer to a SHA2_OperationProcessHash struct */
    SHA2_OperationFinishHash        *finishHash;    /*!< A pointer to a SHA2_OperationFinishHash struct */
    SHA2_OperationOneStepHash       *oneStepHash;   /*!< A pointer to a SHA2_OperationOneStepHash struct */
} SHA2_Operation;

/*!
 *  @brief  Enum for the operation types supported by the driver.
 */
typedef enum SHA2_OperationType_ {
    SHA2_OPERATION_TYPE_START_HASH = 1,
    SHA2_OPERATION_TYPE_PROCESS_HASH = 2,
    SHA2_OPERATION_TYPE_FINISH_HASH = 3,
    SHA2_OPERATION_TYPE_ONE_STEP_HASH = 4,
} SHA2_OperationType;

/*!
 *  @brief SHA2 Global configuration
 *
 *  The SHA2_Config structure contains a set of pointers used to characterize
 *  the SHA2 driver implementation.
 *
 *  This structure needs to be defined before calling SHA2_init() and it must
 *  not be changed thereafter.
 *
 *  @sa     SHA2_init()
 */
typedef struct SHA2_Config_ {
    /*! Pointer to a driver specific data object */
    void               *object;

    /*! Pointer to a driver specific hardware attributes structure */
    void         const *hwAttrs;
} SHA2_Config;

/*!
 *  @brief  The definition of a callback function used by the SHA2 driver
 *          when used in ::SHA2_RETURN_BEHAVIOR_CALLBACK
 *
 *  @param  handle Handle of the client that started the SHA2 operation.
 *
 *  @param  returnStatus The result of the SHA2 operation. May contain an error code.
 *                       Informs the application of why the callback function was
 *                       called.
 *
 *  @param  operation A union of pointers to operation structs. Only one type
 *          of pointer is valid per call to the callback function. Which type
 *          is currently valid is determined by /c operationType. The union
 *          allows easier access to the struct's fields without the need to
 *          typecast the result.
 *
 *  @param  operationType This parameter determines which operation the
 *          callback refers to and which type to access through /c operation.
 */
typedef void (*SHA2_CallbackFxn) (SHA2_Handle handle,
                                  int_fast16_t returnStatus,
                                  SHA2_Operation operation,
                                  SHA2_OperationType operationType);

/*!
 *  @brief  SHA2 Parameters
 *
 *  SHA2 Parameters are used to with the SHA2_open() call. Default values for
 *  these parameters are set using SHA2_Params_init().
 *
 *  @sa     SHA2_Params_init()
 */
typedef struct SHA2_Params_ {
    SHA2_ReturnBehavior     returnBehavior;             /*!< Blocking, callback, or polling return behavior */
    SHA2_CallbackFxn        callbackFxn;                /*!< Callback function pointer */
    uint32_t                timeout;                    /*!< Timeout before the driver returns an error in
                                                         *   ::SHA2_RETURN_BEHAVIOR_BLOCKING
                                                         */
    void                   *custom;                     /*!< Custom argument used by driver
                                                         *   implementation
                                                         */
} SHA2_Params;

/*!
 *  @brief Default SHA2_Params structure
 *
 *  @sa     SHA2_Params_init()
 */
extern const SHA2_Params SHA2_defaultParams;

/*!
 *  @brief  This function initializes the SHA2 module.
 *
 *  @pre    The SHA2_config structure must exist and be persistent before this
 *          function can be called. This function must also be called before
 *          any other SHA2 driver APIs. This function call does not modify any
 *          peripheral registers.
 */
void SHA2_init(void);

/*!
 *  @brief  Function to initialize the SHA2_Params struct to its defaults
 *
 *  @param  params      A pointer to SHA2_Params structure for
 *                      initialization
 *
 *  Defaults values are:
 *      returnBehavior              = SHA2_RETURN_BEHAVIOR_BLOCKING
 *      callbackFxn                 = NULL
 *      timeout                     = SemaphoreP_WAIT_FOREVER
 *      custom                      = NULL
 */
void SHA2_Params_init(SHA2_Params *params);

/*!
 *  @brief  This function opens a given SHA2 peripheral.
 *
 *  @pre    SHA2 controller has been initialized using SHA2_init()
 *
 *  @param  index         Logical peripheral number for the SHA2 indexed into
 *                        the SHA2_config table
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values.
 *
 *  @return A SHA2_Handle on success or a NULL on an error or if it has been
 *          opened already.
 *
 *  @sa     SHA2_init()
 *  @sa     SHA2_close()
 */
SHA2_Handle SHA2_open(uint_least8_t index, SHA2_Params *params);

/*!
 *  @brief  Function to close a SHA2 peripheral specified by the SHA2 handle
 *
 *  @pre    SHA2_open() has to be called first.
 *
 *  @param  handle A SHA2 handle returned from SHA2_open()
 *
 *  @sa     SHA2_open()
 */
void SHA2_close(SHA2_Handle handle);

/*!
 *  @brief  Function performs implementation specific features on a given
 *          SHA2_Handle.
 *
 *  Commands for SHA2_control can originate from SHA2.h or from implementation
 *  specific SHA2*.h (_SHA2CC26XX.h_, _SHA2MSP432.h_, etc.. ) files.
 *  While commands from SHA2.h are API portable across driver implementations,
 *  not all implementations may support all these commands.
 *  Conversely, commands from driver implementation specific SHA2*.h files add
 *  unique driver capabilities but are not API portable across all SHA2 driver
 *  implementations.
 *
 *  Commands supported by SHA2.h follow an SHA2_CMD_\<cmd\> naming
 *  convention.<br>
 *  Commands supported by SHA2*.h follow an SHA2*_CMD_\<cmd\> naming
 *  convention.<br>
 *  Each control command defines @b arg differently. The types of @b arg are
 *  documented with each command.
 *
 *  See @ref SHA2_CMD "SHA2_control command codes" for command codes.
 *
 *  See @ref SHA2_STATUS "SHA2_control return status codes" for status codes.
 *
 *  @pre    SHA2_open() has to be called first.
 *
 *  @param  handle      A SHA2 handle returned from SHA2_open()
 *
 *  @param  cmd         SHA2.h or SHA2*.h commands.
 *
 *  @param  args        An optional R/W (read/write) command argument
 *                      accompanied with cmd
 *
 *  @return Implementation specific return codes. Negative values indicate
 *          unsuccessful operations.
 *
 *  @sa     SHA2_open()
 */
int_fast16_t SHA2_control(SHA2_Handle handle, uint32_t cmd, void *args);

/*!
 *  @brief  Function to initialize a SHA2_OperationStartHash struct to its defaults
 *
 *  @param  operation   A pointer to SHA2_OperationStartHash structure for
 *                      initialization
 *
 *  Defaults values are all zeros.
 */
void SHA2_OperationStartHash_init(SHA2_OperationStartHash *operation);

/*!
 *  @brief  Function to initialize a SHA2_OperationProcessHash struct to its defaults
 *
 *  @param  operation   A pointer to SHA2_OperationProcessHash structure for
 *                      initialization
 *
 *  Defaults values are all zeros.
 */
void SHA2_OperationProcessHash_init(SHA2_OperationProcessHash *operation);

/*!
 *  @brief  Function to initialize a SHA2_OperationFinishHash struct to its defaults
 *
 *  @param  operation   A pointer to SHA2_OperationFinishHash structure for
 *                      initialization
 *
 *  Defaults values are all zeros.
 */
void SHA2_OperationFinishHash_init(SHA2_OperationFinishHash *operation);

/*!
 *  @brief  Function to initialize a SHA2_OperationOneStepHash struct to its defaults
 *
 *  @param  operation   A pointer to SHA2_OperationOneStepHash structure for
 *                      initialization
 *
 *  Defaults values are all zeros.
 */
void SHA2_OperationOneStepHash_init(SHA2_OperationOneStepHash *operation);

/*!
 *  @brief  Sets up a SHA2 hash and hashes the first message segment.
 *
 *  @pre    SHA2_open() has to be called first.
 *
 *  @param  handle A SHA2 handle returned from SHA2_open()
 *
 *  @param  operation   A pointer to a struct containing the requisite
 *                      parameters to execute the function.
 *
 *  @sa     SHA2_open()
 *  @sa     SHA2_processHash()
 *  @sa     SHA2_finishHash()
 */
int_fast16_t SHA2_startHash(SHA2_Handle handle, SHA2_OperationStartHash *operation);

/*!
 *  @brief  Performs the SHA2 hash on a segment of the message.
 *
 *  SHA2_processHash() should be used when only part of a message is available for
 *  hashing.
 *
 *  @pre    SHA2_startHash() must be called before a SHA2_processHash may be called.
 *
 *  \note SHA2_processHash() should NOT be used for the final block of the hash.
 *  If the remainder of the is available for hashing, SHA2_processHash() should
 *  not be called, and the entire message may be hashed with SHA2_finishHash() instead.
 *
 *  @param  handle  A SHA2 handle returned from SHA2_open()
 *
 *  @param  operation   A pointer to a struct containing the requisite
 *                      parameters to execute the function.
 *
 *  @sa     SHA2_startHash()
 *  @sa     SHA2_finishHash()
 */
int_fast16_t SHA2_processHash(SHA2_Handle handle, SHA2_OperationProcessHash *operation);

/*!
 *  @brief  Completes final block(s) of hash and returns a final output hash value.
 *
 *  SHA2_finishHash() must be used to process the final block of the hash, and may also
 *  be used to process some or all preceding blocks of the message.
 *
 *  @pre    SHA2_startHash() must be called before a SHA2_finishHash() may be called;
 *  SHA2_processHash() may optionally be called before SHA2_finishHash().
 *
 *  @param  handle  A SHA2 handle returned from SHA2_open().
 *
 *  @param  operation   A pointer to a struct containing the requisite
 *                      parameters to execute the function.
 *
 *  @sa     SHA2_startHash()
 *  @sa     SHA2_processHash()
 */
int_fast16_t SHA2_finishHash(SHA2_Handle handle, SHA2_OperationFinishHash *operation);

/*!
 *  @brief  Hashes an input message.
 *
 *  SHA2_oneStepHash() hashes a message stored contiguously. Use this function if you
 *  have access to the entire message when starting to hash.
 *
 *  @pre    SHA2_open() must be called prior.
 *
 *  @param  handle      A SHA2 handle returned from SHA2_open().
 *
 *  @param  operation   A pointer to a struct containing the requisite
 *                      parameters to execute the function.
 */
int_fast16_t SHA2_oneStepHash(SHA2_Handle handle, SHA2_OperationOneStepHash *operation);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_SHA2__include */
