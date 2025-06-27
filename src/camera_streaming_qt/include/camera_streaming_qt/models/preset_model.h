#ifndef PRESET_MODEL_H
#define PRESET_MODEL_H

#include <vector>

#include "models/source_model.h"

class Preset {
 public:
  Preset(std::vector<Source*> sources) { sources_ = sources; }

  std::vector<Source*> sources_;
};

#endif