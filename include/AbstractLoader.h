#ifndef ABSTRACTLOADER_H
#define ABSTRACTLOADER_H

#include "MogsProblemClassifier.h"

#include "AbstractConstraint.h"
#include "AbstractCriteria.h"
#include "AbstractParameterization.h"

class AbstractLoader
{
    public:
        AbstractLoader()
        {

        }

        virtual ~AbstractLoader()
        {

        }

        template <typename CREATOR>
        AbstractConstraint * get_constraint(const QString& derivative_type,
                                            const QString& type)
        {
            QString library_so;
            if ( mpc.get_library_plugin(derivative_type,type,library_so))
            {
                // load the library
                void * library = dlopen(library_so.toAscii(), RTLD_LAZY);
                if (!library) {
                    std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
                    exit(0);
                }
//                // load the symbols
                CREATOR creator = (CREATOR) dlsym(library, "create");
//                destructor = (destroy_FAD_1_4Constraint*) dlsym(library, "destroy");
                if (!creator )
                {
                    std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
                    exit(0);
                }
                // create an instance of the class
                return  (AbstractConstraint*) creator();
            }
            else
            {
                qDebug()<<"Error cannot load the plugin "<<type<<" as a "<<derivative_type<<" plugin";
                exit(0);
            }
        }

        template <typename CREATOR>
        AbstractConstraint * get_constraint(  const QString &derivative_type,
                                                    AbstractConstraint* ctr)
        {
            QString type = ctr->get_plugin_name();
            AbstractConstraint * c = get_constraint<CREATOR>(derivative_type,type);
            c->init_from_AbstractConstraint(ctr);
            return c;
        }

        template <typename CREATOR>
        AbstractConstraint * get_constraint(const QString &derivative_type,
                                            QDomElement constraint,
                                            std::vector<MogsOptimDynamics<double> *>& dyns )
        {
            QString type=constraint.attribute("type");
            AbstractConstraint * c = get_constraint<CREATOR>(derivative_type,type);
            c->init_from_xml(constraint,dyns);
            return c;
        }

        template <typename CREATOR>
        AbstractCriteria * get_criteria(    const QString & derivative_type,
                                            const QString & type)
        {
            QString library_so;
			if ( mpc.get_library_plugin(derivative_type,type,library_so))
			{
				// load the library
				void * library = dlopen(library_so.toAscii(), RTLD_LAZY);
				if (!library) {
					std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
					exit(0);
				}
				// load the symbols
				CREATOR creator = (CREATOR) dlsym(library, "create");
//				destructor = (destroy_FAD_1_4Critere*) dlsym(library, "destroy");
				/// FIXME store the destructor !!
				if (!creator /*|| !destructor*/)
				{
					std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
					exit(0);
				}
				// create an instance of the class
				return (AbstractCriteria*) creator();
			}
			else
			{
				qDebug()<<"Error cannot load the plugin "<<type<<" as a "<<derivative_type<<" plugin";
				exit(0);
			}
        }

        template <typename CREATOR>
        AbstractCriteria * get_criteria(    const QString & derivative_type,
                                            QDomElement critere,
                                            std::vector<MogsOptimDynamics<double> *>& dyns )
        {
            QString type=critere.attribute("type");
            AbstractCriteria * c = get_criteria<CREATOR>(derivative_type,type);
            c->init_from_xml(critere,dyns);
            return c;
        }

        template <typename CREATOR>
        AbstractCriteria * get_criteria(const QString& derivative_type,
                                        AbstractCriteria* crit)
        {
            QString type = crit->get_plugin_name();
            AbstractCriteria * p = get_criteria<CREATOR>(derivative_type,type);
            p->init_from_AbstractCriteria(crit);
            return p;
        }

       template <typename CREATOR>
        AbstractParameterization * get_parameterization(const QString& derivative_type,
                                                        const QString& type)
        {
            QString library_so;
            if ( mpc.get_library_plugin(derivative_type,type,library_so))
            {
                // load the library
                void * library = dlopen(library_so.toAscii(), RTLD_LAZY);
                if (!library) {
                    std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load library ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
                    exit(0);
                }
                // load the symbols
                CREATOR creator = (CREATOR) dlsym(library, "create");
//                destructor = (destroy_FAD_1_4Parameterization*) dlsym(library, "destroy");
                if (!creator /*|| !destructor*/)
                {
                    std::cerr <<"Error in "<<__FILE__<<" at line "<<__LINE__<< " : Cannot load symbols of ("<< library_so.toStdString()<<"), with the error : " << dlerror() << '\n';
                    exit(0);
                }
                // std::cout << "loading parameterization_ name "   <<type.toStdString().c_str() << std::endl;
                // create an instance of the class
                return( AbstractParameterization *) creator();
            }
            else
            {
                qDebug()<<"Error cannot load the plugin "<<type<<" as a "<<derivative_type<<" plugin";
                exit(0);
            }
        }

        template <typename CREATOR>
        AbstractParameterization * get_parameterization(const QString& derivative_type,
                                                        AbstractParameterization* param)
        {
            QString type = param->get_plugin_name();
            AbstractParameterization * p = get_parameterization<CREATOR>(derivative_type,type);
            p->init_from_AbstractParameterization(param);
            return p;
        }

        template <typename CREATOR>
        AbstractParameterization * get_parameterization(const QString& derivative_type,
                                                        QDomElement param,
                                                        std::vector<MogsOptimDynamics<double> *>& dyns )
        {
            QString type=param.attribute("type");
            AbstractParameterization * p = get_parameterization<CREATOR>(derivative_type,type);
            p->init_from_xml(param,dyns);
            return p;
        }



    protected:
    private:

        MogsProblemClassifier mpc;
};

#endif // ABSTRACTLOADER_H
