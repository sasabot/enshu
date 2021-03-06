#ifndef _NEGOMO_MODEL_DATA_
#define _NEGOMO_MODEL_DATA_

#include <vector>
#include <string>

namespace negomo
{
  namespace model_data
  {
    struct data
    {
      std::vector<std::string> observations;
      std::vector<std::vector<char> > sequences;
      char final_state;
      int length;
    };

    const static std::vector<std::string> param_name =
    {
	"/negotiation_model/k_initial70", "/negotiation_model/k_during70"
      // write parameter names
    };

    const static std::vector<std::string> filenames =
    {
	"initial70.hmm", "during70.hmm"
      // write filenames
    };
    
    const static std::vector<std::vector<data> > model_data
    {
      // implement data
    };

  }
}

#endif
