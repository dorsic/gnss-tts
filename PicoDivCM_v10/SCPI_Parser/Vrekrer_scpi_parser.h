/*! 
@file Vrekrer_scpi_parser.h 
Header file.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#ifndef VREKRER_SCPI_PARSER_H_
#define VREKRER_SCPI_PARSER_H_


/// Max branch size of the command tree and max number of parameters.
#ifndef SCPI_ARRAY_SIZE
  #define SCPI_ARRAY_SIZE 6
#endif

/// Max number of valid tokens.
#ifndef SCPI_MAX_TOKENS
  #define SCPI_MAX_TOKENS 12
#endif

/// Max number of registered commands.
#ifndef SCPI_MAX_COMMANDS
  #define SCPI_MAX_COMMANDS 20
#endif

/// Length of the message buffer.
#ifndef SCPI_BUFFER_LENGTH
  #define SCPI_BUFFER_LENGTH 64
#endif

/// Timeout, in microseconds, for GetMessage and ProcessInput.
#ifndef SCPI_TIMEOUT
  #define SCPI_TIMEOUT 10000
#endif

/// Integer size used for hashes.
#ifndef SCPI_HASH_TYPE
  #define SCPI_HASH_TYPE uint8_t
#endif

/*!
 Variable size string array class.

 The array must be filled using the \c Append method (acts as a LIFO stack Push). \n
 Values can be extracted (and removed) using the \c Pop function (LIFO stack Pop). \n
 Both \c Append and \c Pop modifies the \c Size of the array. \n
 Values can be read (without removing them) using the following methods: \n
  \li \c First() : Returns the first value appended to the array.
  \li \c Last()  : Returns the last value appended to the array.
  \li Indexing (e.g. \c my_array[1] to get the second value of the array).

 The max size of the array is defined by \c SCPI_ARRAY_SIZE (default 6).
*/
class SCPI_String_Array {
 public:
  char* operator[](const uint8_t index);  //Add indexing capability
  void Append(char* value);            //Append new string (LIFO stack Push)
  char* Pop();                         //LIFO stack Pop
  char* First();                       //Returns the first element of the array
  char* Last();                        //Returns the last element of the array
  uint8_t Size();                      //Array size
 protected:
  uint8_t size_ = 0;              //Internal array size
  char* values_[SCPI_ARRAY_SIZE]; //Storage of the strings
};

/*!
 String array class used to store the tokens of a command.
 @see SCPI_String_Array
*/
class SCPI_Commands : public SCPI_String_Array {
 public:
  //Dummy constructor.
  SCPI_Commands();
  //Constructor that extracts and tokenize a command from a message
  SCPI_Commands(char* message);
  ///Not processed part of the message after the constructor is called.
  char* not_processed_message;
};

/*!
 String array class used to store the parameters found after a command.
 @see SCPI_String_Array
*/
class SCPI_Parameters : public SCPI_String_Array {
 public:
  //Dummy constructor.
  SCPI_Parameters();
  //Constructor that extracts and splits parameters from a message
  SCPI_Parameters(char *message);
  ///Not processed part of the message after the constructor is called.
  char* not_processed_message;
};

class SCPI_Interface {
  public:
    //Constructor
    SCPI_Interface();
    char getchar_non_blocking();
    void putchars(char* message);
};

///Alias of SCPI_Commands.
typedef SCPI_Commands SCPI_C;

///Alias of SCPI_Parameters.
typedef SCPI_Parameters SCPI_P;

///Alias of SCPI_Interface.
typedef SCPI_Interface SCPI_I;

///Void template used with SCPI_Parser::RegisterCommand.
typedef void (*SCPI_caller_t)(SCPI_Commands, SCPI_Parameters, SCPI_Interface);


class SCPI_Printf_Interface : public SCPI_Interface {
  public:
    //Constructor
    SCPI_Printf_Interface();
    char getchar_non_blocking();
    void putchars(char* message);
//    ~SCPI_Printf_Interface();
};


class SCPI_SPI_Interface : public SCPI_Interface {
  public:
    //Constructor
    SCPI_SPI_Interface(spi_inst_t *spi, uint8_t cs);
    char getchar_non_blocking();
    void putchars(char* message);
  protected:
    spi_inst_t *spi_;
    uint8_t cs_;
    uint8_t buf_[32];
    uint8_t bufidx_ = 0;
};


/*!
  Main class of the Vrekrer_SCPI_Parser library.
*/
class SCPI_Parser {
 public:
  //Constructor
  SCPI_Parser();
  //Change the TreeBase for the next RegisterCommand calls
  void SetCommandTreeBase(char* tree_base);
  //SetCommandTreeBase version with RAM string support
  void SetCommandTreeBase(const char* tree_base);
  //SetCommandTreeBase version with Flash strings (F() macro) support
  //void SetCommandTreeBase(const __FlashStringHelper* tree_base);
  //Registers a new valid command and associate a procedure to it
  void RegisterCommand(char* command, SCPI_caller_t caller);
  //RegisterCommand version with RAM string support.
  void RegisterCommand(const char* command, SCPI_caller_t caller);
  //RegisterCommand version with Flash strings (F() macro) support
  //void RegisterCommand(const __FlashStringHelper* command, SCPI_caller_t caller);
  //Set the function to be used by the error handler.
  void SetErrorHandler(SCPI_caller_t caller);
  ///SCPI Error codes.
  enum class ErrorCode{
    ///No error
    NoError,
    ///Unknown command received.
    UnknownCommand,
    ///Timeout before receiving the termination chars.
    Timeout,
    ///Message buffer overflow.
    BufferOverflow,
    ///CommandOverflow,
    CommandOverflow
  };
  ///Variable that holds the last error code.
  ErrorCode last_error = ErrorCode::NoError;
  //Process a message and execute it a valid command is found
  void Execute(SCPI_Interface interface, char* message);
  //Gets a message from a Stream interface and execute it
  void ProcessInput(SCPI_Interface interface, const char* term_chars);
  //Gets a message from a Stream interface
  char* GetMessage(SCPI_Interface interface, const char* term_chars);
  //Prints registered tokens and command hashes to the serial interface
  void PrintDebugInfo();
  ///Magic number used for hashing the commands
  SCPI_HASH_TYPE hash_magic_number = 37;
  
 protected:
  //Add a token to the tokens' storage
  void AddToken_(char* token);
  //Get a hash from a command
  SCPI_HASH_TYPE GetCommandCode_(SCPI_Commands& commands);
  //Number of stored tokens
  uint8_t tokens_size_ = 0;
  //Storage for tokens
  char *tokens_[SCPI_MAX_TOKENS];
  //Number of registered commands
  uint8_t codes_size_ = 0;
  //Registered commands' hash storage
  SCPI_HASH_TYPE valid_codes_[SCPI_MAX_COMMANDS];
  //Pointers to the functions to be called when a valid command is received
  SCPI_caller_t callers_[SCPI_MAX_COMMANDS+1];
  //Branch's hash used when calculating unique codes (0 for root)
  SCPI_HASH_TYPE tree_code_ = 0;
  //Message buffer.
  char msg_buffer_[SCPI_BUFFER_LENGTH];
  //Length of the readed message
  uint8_t message_length_ = 0;
  //Varible used for checking timeout errors
  uint32_t time_checker_;
};

#endif //VREKRER_SCPI_PARSER_H_
