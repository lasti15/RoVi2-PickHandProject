

#include "GraspDBRepository.hpp"

#include <rw/common/DOMParser.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <rw/common.hpp>

using namespace rwlibs::task;
using namespace rw::common;

namespace {


}

GraspDBRepository::GraspDBRepository():_repositoryPath(""){

}

GraspDBRepository::GraspDBRepository(const std::string& path):
		_repositoryPath(path)
{

}

void GraspDBRepository::initialize(){
	if(_repositoryPath.empty())
		_repositoryPath = ".";
	// check if there is a configuration file in the root of the graspDG repository
	if(!boost::filesystem::exists(_repositoryPath)){
		RW_THROW("Invalid repository path! It does not exist: " << _repositoryPath );
	}
	if( boost::filesystem::exists(_repositoryPath+"/graspDB_config.xml") ){
		RW_WARN("1");
		_parser = DOMParser::Factory::getDOMParser("xml");
		_parser->load(_repositoryPath+"/graspDB_config.xml");
	} else {
		_parser = DOMParser::Factory::getDOMParser("xml");
		_parser->getRootElement()->addChild("GraspDBRepository");
		_parser->save( _repositoryPath+"/graspDB_config.xml" );
	}

	// scan the directories
	std::vector<std::string> grippers = getGrippers();
	std::cout << "Grippers currently available in repository: " << std::endl;
	BOOST_FOREACH(std::string gripper, grippers){
		std::cout << gripper << ", ";
	}
	std::cout << "\n";

	std::cout << "Gripper/objects currently available in repository: " << std::endl;
	BOOST_FOREACH(std::string gripper, grippers){
		std::cout << gripper << " : ";
		std::vector<std::string> objects = getObjects(gripper);
		BOOST_FOREACH(std::string object, objects){
			std::cout << object << ", ";
		}
		std::cout << "\n";
	}

}

void GraspDBRepository::setRepositoryPath(const std::string& path){
	_repositoryPath = path;
}

const std::string& GraspDBRepository::getRepositoryPath(){
	return _repositoryPath;
}

std::vector<std::string> GraspDBRepository::getGrippers(){
    std::vector<std::string> dirs;
    boost::filesystem::directory_iterator end;
    for (boost::filesystem::directory_iterator it(_repositoryPath); it != end; it++)
    {
        if (boost::filesystem::is_directory(it->status())) {
        	dirs.push_back( it->path().filename().string() );
        }
    }
    return dirs;
}

std::vector<std::string> GraspDBRepository::getObjects(const std::string& grippername){
    std::vector<std::string> objects;
    std::string path = _repositoryPath+"/"+grippername;
    std::set<std::string> uniqueNames;
    boost::filesystem::directory_iterator end;
    for (boost::filesystem::directory_iterator it(path); it != end; it++)
    {
        if (!boost::filesystem::is_directory(it->status())) {
        	std::string filename = it->path().filename().string();
        	// split the name to get the object part
        	std::vector<std::string> strs;
        	boost::split(strs,filename,boost::is_any_of("_"));
        	uniqueNames.insert(strs[0]);

        }
    }
    BOOST_FOREACH(std::string name, uniqueNames){
    	objects.push_back(name);
    }
    return objects;
}

std::vector<std::pair<GraspDBRepository::Context, std::string> >
GraspDBRepository::getContexts(const std::string& grippername, const std::string& objectname)
{
    std::vector<std::pair<GraspDBRepository::Context, std::string> > contexts;
    std::string path = _repositoryPath+"/"+grippername;
    std::vector<std::string> files = IOUtil::getFilesInFolder(path,false, false, objectname+"_*");
    BOOST_FOREACH(std::string file, files){
    	std::vector<std::string> strs;
    	boost::split(strs,file,boost::is_any_of("_"));
    	contexts.push_back( std::make_pair(GraspDBRepository::FLOAT, strs[1]) );
    }
    return contexts;
}

std::vector<rwlibs::task::GraspTask::Ptr> GraspDBRepository::getGrasps(const std::string& grippername, const std::string& objectname, std::vector<std::string> objectalises){
	std::vector<GraspTask::Ptr> gtasks = getGrasps(grippername,objectname);

	BOOST_FOREACH(std::string objname, objectalises){
		if(gtasks.size()>0){
			return gtasks;
		}
		gtasks = getGrasps(grippername,objname);
	}
	return gtasks;
}

std::vector<GraspTask::Ptr> GraspDBRepository::getGrasps(const std::string& grippername, const std::string& objectname)
{
    std::vector<GraspTask::Ptr> gtasks;

    std::string path = _repositoryPath+"/"+grippername;
    std::vector<std::string> files = IOUtil::getFilesInFolder(path,false, true, objectname+"_*");
    Log::debugLog() << "Loading " << files.size() << " grasp task files." << std::endl;
    Timer time;
    // try loading each of the files as task files
    BOOST_FOREACH(std::string file, files){
    	GraspTask::Ptr gtask = GraspTask::load(file);
    	gtasks.push_back(gtask);
    }
    Log::debugLog() << "Loading took " << time.toString() << std::endl;
    return gtasks;
}

std::vector<GraspTask::Ptr> GraspDBRepository::getGrasps(const std::string& grippername, const std::string& objectname, Context context)
{
    std::vector<GraspTask::Ptr> gtasks;


    std::string path = _repositoryPath+"/"+grippername;
    std::vector<std::string> files = IOUtil::getFilesInFolder(path,false, true, objectname+"_"+ContextToStr(context)+"_*");
    Log::debugLog() << "Loading " << files.size() << " grasp task files." << std::endl;
    rw::common::Timer time;
    // try loading each of the files as task files
    BOOST_FOREACH(std::string file, files){
    	GraspTask::Ptr gtask = GraspTask::load(file);
    	gtasks.push_back(gtask);
    }
    Log::debugLog() << "Loading took " << time.toString() << std::endl;
    return gtasks;
}

std::vector<GraspTask::Ptr> GraspDBRepository::getGrasps(const std::string& grippername, const std::string& objectname, const std::string& specialContextName)
{
    std::vector<GraspTask::Ptr> gtasks;
    std::string path = _repositoryPath+"/"+grippername;
    std::vector<std::string> files = IOUtil::getFilesInFolder(path,false, true, objectname+"_SPECIAL-"+specialContextName+"_*");
    Log::debugLog() << "Loading " << files.size() << " grasp task files." << std::endl;
    Timer time;
    // try loading each of the files as task files
    BOOST_FOREACH(std::string file, files){
    	GraspTask::Ptr gtask = GraspTask::load(file);
    	gtasks.push_back(gtask);
    }
    Log::debugLog() << "Loading took " << time.toString() << std::endl;
    return gtasks;
}

void GraspDBRepository::addGrasps(const std::string& grippername, const std::string& objectname, Context context, GraspTask::Ptr gtask)
{
	// check if directory exist
	if(!boost::filesystem::exists(_repositoryPath+"/"+grippername)){
		// create directory
		boost::filesystem::create_directory( _repositoryPath+"/"+grippername );
	}

	std::vector<std::string> files = IOUtil::getFilesInFolder(_repositoryPath+"/"+grippername,false, false, objectname+"_"+ContextToStr(context)+"_*");
	// now make sure to find an available id, we just pick the largest available
	int largestId = 0;
	BOOST_FOREACH(std::string file, files){
		std::vector<std::string> strs;
		boost::split(strs,file,boost::is_any_of("_."));
		int val = boost::lexical_cast<int>(strs[2]);
		if(val>largestId)
			largestId = val;
	}
	largestId++;
	// now save the grasp db
	std::stringstream sstr;
	sstr << _repositoryPath<<"/"<<grippername<<"/"<<objectname<<"_"<<ContextToStr(context)<<"_"<<largestId<<".xml";
	GraspTask::saveRWTask(gtask, sstr.str() );
}
