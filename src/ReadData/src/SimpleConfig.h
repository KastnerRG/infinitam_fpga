/*
 * SimpleConfigFile.h
 *
 *  Created on: Aug 24, 2016
 *      Author: qkgautier
 */

#ifndef SRC_SIMPLECONFIG_H_
#define SRC_SIMPLECONFIG_H_

#include <map>
#include <set>
#include <string>
#include <fstream>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <vector>


class SimpleConfig
{
public:
	void loadConfig(const std::string& filename)
	{
	    std::ifstream input(filename);

		if(!input.is_open())
		{
			throw new std::runtime_error(filename + " does not exist.");
		}

	    while(input)
	    {
	        std::string key;
	        std::string value;
	        std::getline(input, key, '=');
	        if(input.bad() || input.eof()){ break; }
	        std::getline(input, value, '\n');

	        // Get part between quotes
	        std::string::size_type pos1 = value.find_first_of("\"");
	        std::string::size_type pos2 = value.find_last_of("\"");
	        if(pos1 != std::string::npos && pos2 != std::string::npos && pos2 > pos1)
	        {
	            value = value.substr(pos1+1,pos2-pos1-1);
	        }
			// Or the trimmed string otherwise
			else{ trim(value); }
	        trim(key);
	        config_[key] = value;
	    }
	}

	std::string getValue(const std::string& key){ queriedKeys_.insert(key); return config_[key]; }

	template<class T> void getValue(const std::string& key, T& value){ std::stringstream(getValue(key)) >> value; }

	void getUnqueriedKeys(std::vector<std::string>& keys)
	{
		keys.clear();
		keys.reserve(config_.size() - queriedKeys_.size());
		for(auto const& element: config_)
		{
			bool found = queriedKeys_.count(element.first) > 0;
			if(!found){ keys.push_back(element.first); }
		}
	}

private:
	std::map<std::string, std::string> config_;
	std::set<std::string> queriedKeys_;


private:
	static inline std::string &ltrim(std::string &s) {
	    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
	            std::not1(std::ptr_fun<int, int>(std::isspace))));
	    return s;
	}

	static inline std::string &rtrim(std::string &s) {
	    s.erase(std::find_if(s.rbegin(), s.rend(),
	            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
	    return s;
	}

	static inline std::string &trim(std::string &s) {
	    return ltrim(rtrim(s));
	}
};




#endif /* SRC_SIMPLECONFIG_H_ */
