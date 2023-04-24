#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include "timeutil.hpp"
#include "util.hpp"

class Owl;
extern Owl owlInstance;

#define OwlInit( settings ) owlInstance.init( settings );
#define OwlShutdown() owlInstance.shutdown();
#define owl (owlInstance << owlInstance.prefix( __FILE__ , __LINE__ ))

class OwlSettings
{
public:
	// path to the location of the log file, working directory by default
	std::string path;
	// filename, default "owl.log"
	std::string filename;
	// the log file will be overwritten, if false a new log file with the date and time is created
	bool b_overwriteFile;

	// print to the log file
	bool b_file;
	// print to the standard output
	bool b_stdout;

	// print the date and time
	bool b_datetime;
	// print the file and line
	bool b_fileline;

	OwlSettings()
	{
		this->path = "";
		this->filename = "owl.log";
		this->b_overwriteFile = true;
		this->b_file = true;
		this->b_stdout = true;
		this->b_datetime = true;
		this->b_fileline = true;
	}
};

class Owl
{
public:
	Owl();
	~Owl();

	bool init( const OwlSettings& settings );
	void shutdown();

	// overload the << operator to log all standard data types
	template< typename T > Owl& operator<<( T t );
	// this is needed so << std::endl works
	Owl& operator<<( std::ostream& (*fun)( std::ostream& ) );

	// creates the prefix for every line depending on the settings
	std::string prefix( const std::string& file , const int line );

private:
	bool b_init;
	OwlSettings settings;
	std::ofstream ofs;
};

template< typename T > inline Owl& Owl::operator<<( T t )
{
	if ( this->b_init )
	{
		if ( this->settings.b_file ) this->ofs << t;
		if ( this->settings.b_stdout ) std::cout << t;
	}
	return *this;
}

inline Owl& Owl::operator<<( std::ostream& (*fun)( std::ostream& ) )
{
	if ( this->b_init )
	{
		if ( this->settings.b_file ) this->ofs << std::endl;
		if ( this->settings.b_stdout ) std::cout << std::endl;
	}
	return *this;
}