#include "OptionModule.h"
#include <param.h>
#include "WorldModel.h"

namespace{
	bool IS_LEFT_SIDE = false;
	bool IS_YELLOW_TEAM = false;
}

COptionModule::COptionModule()
{
	DECLARE_PARAM_READER_BEGIN(General)
	READ_PARAM(IS_LEFT_SIDE)
	READ_PARAM(IS_YELLOW_TEAM)
	DECLARE_PARAM_READER_END
	if (IS_LEFT_SIDE) {
		_side = Param::Field::POS_SIDE_LEFT;
	} else{
		_side = Param::Field::POS_SIDE_RIGHT;
	}

	if (IS_YELLOW_TEAM) {
		_color = TEAM_YELLOW;
	} else{
		_color = TEAM_BLUE;
	}
	std::cout << "Side : " << ((_side == Param::Field::POS_SIDE_LEFT) ? "left" : "right")
			  << ", Color : " << ((_color == TEAM_YELLOW) ? "yellow" : "blue") << " is running..." << Param::Output::NewLineCharacter;
	WorldModel::Instance()->registerOption(this);
}

COptionModule::~COptionModule(void)
{

}