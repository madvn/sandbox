
#include <cmake_template/calculator-cli/calculator-cli.h>
#include <cmake_template/addition/addition.h>

#include <memory>

int main(int argc, char** argv)
{
  cmake_template::calculator::Calculator calc;
  calc.addOperator(std::make_shared<cmake_template::addition::Addition>());

  cmake_template::calculator_cli::CalculatorCli calculatorCli(calc);

  calculatorCli.displayOperators();

  do
  {
    calculatorCli.getFirstNumber();

    calculatorCli.getOperator();

    calculatorCli.getSecondNumber();

    try
    {
      double result = calculatorCli.evaluate();
      calculatorCli.print("Result is: " + std::to_string(result));
    }
    catch (std::exception& e)
    {
      calculatorCli.print("Unable to evaluate the expression: " + std::string(e.what()));
    }

  } while (calculatorCli.keepCalculating());
}