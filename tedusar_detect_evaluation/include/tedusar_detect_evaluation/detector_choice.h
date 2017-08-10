//
// Created by jester on 29.07.15.
//

#ifndef BAC_DETECTORCHOICE_H
#define BAC_DETECTORCHOICE_H

#include <tedusar_detect_evaluation/abstract_choice.h>

#include <vector>
#include <string>

namespace detector_choice
{
  class DetectorChoice : AbstractChoice
  {

  public:
    DetectorChoice();

    virtual ~DetectorChoice();

    virtual int execute();

    unsigned int getParamCnt()
    {
      return 0;
    }
  };
}

#endif //BAC_DETECTORCHOICE_H
