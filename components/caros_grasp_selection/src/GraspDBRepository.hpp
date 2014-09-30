/*
 * GraspDBRepository.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: jimali
 */

#ifndef GRASPDBREPOSITORY_HPP_
#define GRASPDBREPOSITORY_HPP_

#include <rwlibs/task/GraspTask.hpp>
#include <rwlibs/task.hpp>
#include <rw/common/DOMElem.hpp>
#include <rw/common/DOMParser.hpp>
#include <rw/common/Ptr.hpp>

#include <vector>

/**
 * The grasp repository represents a structured directory in which
 * grasptables are structured such as to allow for storing multiple grasp tables
 * for multiple grippers. The grasp repository use gripper name, object name and
 * grasp context name to index grasp tables.
 */
class GraspDBRepository {
public:

	typedef rw::common::Ptr<GraspDBRepository> Ptr;

	typedef enum {FLOAT, //! a generic context without any obstacles or gravity
				  PLANE, //! a context where object is placed on planer surface, gravity working opposite plane normal
				  PLANE_CLUTTERED, //! a context where object is placed on plane surface together with many other objects
				  BIN, //! a context where the object is lying unstructured in a BIN with other objects
				  SPECIAL
	} Context;

	const char* ContextToStr( Context c ){
		if(c==FLOAT){ return "FLOAT"; }
		else if(c==PLANE){ return "PLANE"; }
		else if(c==PLANE_CLUTTERED){ return "PLANE-CLUTTERED"; }
		else if(c==BIN){ return "BIN"; }
		else if(c==SPECIAL){ return "SPECIAL"; }
		else if(c==FLOAT){ return "FLOAT"; }
		return "";
	}

	GraspDBRepository();

	GraspDBRepository(const std::string& path);

	void initialize();

	void setRepositoryPath(const std::string&);
	const std::string& getRepositoryPath();

	std::vector<std::string> getGrippers();
	std::vector<std::string> getObjects(const std::string& grippername);
	std::vector<std::pair<Context, std::string> > getContexts(const std::string& grippername, const std::string& objectname);

	std::vector<rwlibs::task::GraspTask::Ptr> getGrasps(const std::string& grippername, const std::string& objectname, std::vector<std::string> objectaliases);

	std::vector<rwlibs::task::GraspTask::Ptr> getGrasps(const std::string& grippername, const std::string& objectname);
	std::vector<rwlibs::task::GraspTask::Ptr> getGrasps(const std::string& grippername, const std::string& objectname, Context context);
	std::vector<rwlibs::task::GraspTask::Ptr> getGrasps(const std::string& grippername, const std::string& objectname, const std::string& specialContextName);

	void addGrasps(const std::string& grippername, const std::string& objectname, Context context, rwlibs::task::GraspTask::Ptr gtask);

private:
	rw::common::DOMParser::Ptr _parser;
	std::string _repositoryPath;
};



#endif /* GRASPDBREPOSITORY_HPP_ */
