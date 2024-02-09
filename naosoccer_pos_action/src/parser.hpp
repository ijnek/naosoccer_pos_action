// Copyright 2023 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PARSER_HPP_
#define PARSER_HPP_

#include <string>
#include <vector>

#include "naosoccer_pos_action/key_frame.hpp"

namespace parser
{

struct ParseResult
{
  bool successful;
  std::vector<KeyFrame> keyFrames;
};

ParseResult parse(const std::vector<std::string> & in);

}  // namespace parser

#endif  // PARSER_HPP_
