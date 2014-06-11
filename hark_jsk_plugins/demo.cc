#include "BufferedNode.h"
#include "operators.h"

using namespace std;

namespace FD {

class Demo;

DECLARE_NODE(Demo)
/*Node
 *
 * @name Demo
 * @category Demo
 * @description Adds two input values and returns the result
 *
 * @input_name INPUT1
 * @input_description First value
 *
 * @input_name INPUT2
 * @input_description Second value
 *
 * @output_name OUTPUT
 * @output_description Result of the addition
 *
END*/


class Demo : public BufferedNode {

   int m_input1ID;
   int m_input2ID;
   int m_outputID;

public:
   Demo(string nodeName, ParameterSet params)
   : BufferedNode(nodeName, params)
   {
      m_input1ID = addInput("INPUT1");
      m_input2ID = addInput("INPUT2");
      m_outputID = addOutput("OUTPUT");
   }


   void calculate(int output_id, int count, Buffer &out)
   {
      ObjectRef firstValue = getInput(m_input1ID, count);
      ObjectRef secondValue = getInput(m_input2ID, count);

      out[count] = firstValue + secondValue;
   }

   NO_ORDER_NODE_SPEEDUP(Demo)
};
}//namespace FD
