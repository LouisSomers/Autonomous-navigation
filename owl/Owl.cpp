#include "Owl.hpp"

Owl owlInstance;

Owl::Owl()
{
	this->b_init = false;
}

Owl::~Owl()
{
	if ( this->b_init ) this->shutdown();
}

bool Owl::init( const OwlSettings& settings )
{
	this->settings = settings;
	if ( !this->b_init )
	{
		if ( this->settings.b_overwriteFile )
		{
			this->ofs.open( this->settings.path+this->settings.filename );
		}
		else
		{
			this->ofs.open( this->settings.path+getDateTimeFilenameStr()+"_"+this->settings.filename );
		}

		if ( this->ofs.is_open() )
		{
			this->b_init = true;
			return true;
		}
		else
		{
			return false;
		}
	}
	return true;
}

void Owl::shutdown()
{
	if ( this->b_init )
	{
		this->ofs.close();
		this->b_init = false;
	}
}

std::string Owl::prefix( const std::string& file , const int line ) 
{
	std::string result = "";
	if ( this->settings.b_datetime )
	{
		result += getDateTimeStr();

		if ( !this->settings.b_fileline ) result += ": ";
		else result += ", ";
	}
	if ( this->settings.b_fileline )
	{
		result += file + "(" + tostring(line) + "): ";
	}
	return result;
}