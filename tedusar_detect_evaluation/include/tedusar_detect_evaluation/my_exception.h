//
// Created by jester on 24.07.15.
//

#ifndef BAC_MYEXCEPTION_H
#define BAC_MYEXCEPTION_H

#include <exception>
#include <stdexcept>
#include <string>

class MyException : public std::exception
{
public:
  enum ErrorMsg
  {
    OUTOFMANA,
    STARTCHOICE,
    ENDCHOICE,
    WRONG_PARAM_COUNT,
    UNKNOWN_COMMAND,
    IMPOSSIBLE_OPEN_FILE,
    ERROR_LOADING,
    TINY_XML_FAILED,
  };

  //************************************************************************************************
  // Constructor
  // @ param message a collection of error messages
  // @ param additional_info (optional) extension of the error messages
  MyException(ErrorMsg msg, const char *additional_info = NULL);

  virtual ~MyException()  throw() {}

  static const std::string MESSAGES[];

  virtual const char *what();

private:
  const ErrorMsg error_msg_;
  const char *additional_info_;
  std::string err_full_msg_;
};


#endif //BAC_MYEXCEPTION_H
