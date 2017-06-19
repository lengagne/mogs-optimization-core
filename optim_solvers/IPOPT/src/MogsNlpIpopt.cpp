#include <MogsNlpIpopt.hpp>
using namespace Ipopt;

MogsNlpIpopt::MogsNlpIpopt ()
{
}
MogsNlpIpopt::~MogsNlpIpopt ()
{
}
void MogsNlpIpopt::save_results( 	Index n,
									const Number* x,
									Number obj_value)
{
	// For this example, we write the solution to the console
	printf("\n\nSolution of the primal variables, x\n");
	for (Index i=0; i<n; i++) {
	printf("x[%d] = %e\n", i, x[i]);
	}
	printf("\n\nObjective value\n");
	printf("f(x*) = %e\n", obj_value);

	QDomElement ElResults = root_.firstChildElement ("results");

	qDebug()<<"Create node";
	QDomDocument doc = root_.ownerDocument();
	QDomElement node = doc.createElement("result");
	ElResults.appendChild(node);

//	qDebug()<<" fichiers = "<< doc.toString();
// 	root_.
}

void MogsNlpIpopt::set_robots(const std::vector<MogsRobotProperties*> & in)
{
    robots_ = in;
    nb_robots_  = robots_.size();
    std::cout<<"MogsNlpIpopt::set_robots nb_robots_ = "<< nb_robots_<<std::endl;
}
