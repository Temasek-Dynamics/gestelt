
class PlannerBase
{
public:
  PlannerBase(){

  }

  // Adds pointer reference to gridmap
  bool addGridMap(){
    init_ = true;
  }

  bool updateStartAndGoal(){

  }

  bool updateStart(){

  }

  bool updateGoal(){
    
  }

private: 
  bool init_;

};