/*
 * Copyright 2021 SDU - Anders Prier Lindvig anpl@mmmi.sdu.dk (refactor)
 *
 * Copyright 2019 FZI Forschungszentrum Informatik - Tristan Schnell schnell@fzi.de (original)
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
#pragma once
#ifndef URCL_SCRIPT_SENDER_H
#define URCL_SCRIPT_SENDER_H

#include <urcl/server.h>

#include <thread>
#include <iostream>
#include <chrono>

namespace urcl
{
namespace comm
{
/*!
 * \brief The ScriptSender class starts a URServer for a robot to connect to and waits for a
 * request to receive a program. This program is then delivered to the requesting robot.
 */
class ScriptSender
{
 public:
  ScriptSender() = delete;
  /*!
   * \brief Creates a ScriptSender object, including a new URServer and not yet started thread.
   *
   * \param port Port to start the server on
   * \param program Program to send to the robot upon request
   */
  ScriptSender(uint32_t port, const std::string& program, bool verbose = false)
      : server_(port), script_thread_(), program_(program), verbose_(verbose)
  {
    stop_thread_ = false;
    if (!server_.bind())
    {
      throw std::runtime_error("Could not bind to server");
    }
  }

  ~ScriptSender()
  {
    stop_thread_ = true;
    server_.disconnectClient();
    script_thread_.detach();
  }

  /*!
   * \brief Starts the thread that handles program requests by a robot.
   */
  void start()
  {
    script_thread_ = std::thread(&ScriptSender::runScriptSender, this);
  }

 private:
  URServer server_;
  std::thread script_thread_;
  std::string program_;
  bool verbose_;
  std::atomic<bool> stop_thread_{false};

  const std::string PROGRAM_REQUEST_ = std::string("request_program\n");

  void runScriptSender()
  {
    while (!stop_thread_)
    {
      if (!server_.accept())
      {
        throw std::runtime_error("Failed to accept robot connection");
      }
      if (requestRead())
      {
        if (verbose_)
          std::cout << "Robot requested program" << std::endl;
        sendProgram();
      }
      server_.disconnectClient();
    }
  }

  bool requestRead()
  {
    const size_t buf_len = 1024;
    char buffer[buf_len];

    bool read_successful = server_.readLine(buffer, buf_len);

    if (read_successful)
    {
      if (std::string(buffer) == PROGRAM_REQUEST_)
      {
        return true;
      }
      else
      {
        if (verbose_)
          std::cout << "Received unexpected message on script request port" << std::endl;
      }
    }
    else
    {
      if (verbose_)
        std::cout << "Could not read on script request port" << std::endl;
    }
    return false;
  }

  void sendProgram()
  {
    size_t len = program_.size();
    const uint8_t* data = reinterpret_cast<const uint8_t*>(program_.c_str());
    size_t written;

    if (server_.write(data, len, written))
    {
      if (verbose_)
        std::cout << "Sent program to robot" << std::endl;
    }
    else
    {
      std::cerr << "Could not send program to robot" << std::endl;
    }
  }
};

}  // namespace comm
}  // namespace urcl

#endif  // URCL_SCRIPT_SENDER_H
