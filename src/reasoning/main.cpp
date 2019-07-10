#include "ros/ros.h"
#include <string>
#include "prolog/PrologClient.h"
#include "NodeManager.h"


main(int argc, char **argv) {
    ros::init(argc, argv, "reasoning");
    reasoning::PrologClient pc;
    /*pc.test_prolog_query();
    pc.decide_action();
    pc.search_random_card();
    std::cout<<"Finished Testing the core dump is not Henriques problem "<<std::endl;*/
    NodeManager nm;

    boost::circular_buffer<reasoning::State> test(5);
    std::cout<<"First"<<std::endl;
    test.push_back(reasoning::State::EXPOSED)  ;
    test.push_back(reasoning::State::EXPOSED)  ;
    test.push_back(reasoning::State::EXPOSED)  ;
    test.push_back(reasoning::State::EXPOSED)  ;
    test.push_back(reasoning::State::EXPOSED)  ;
    test.push_back(reasoning::State::EXPOSED)  ;
    if (*test.begin() == reasoning::State::EXPOSED){
    std::cout<<"Test done lelel"<<std::endl;
    }
    nm.surrect();

}
    //bool MyTurn=true; //TRUE means NA0, False means opponent
    //int validGame=1;
    /**while(validGame){
        if(MyTurn){
            std::cout<<"Nao is playing"<<std::endl;
            //observe and act and store data
            std::cout<<"Nao is gonn"<<std::endl;
            //Look into memory for equal cards-->Prolog(Rule:act(Card1,Card2)) gives back one of two actions
            //Call findTwoEqualCards-> 2 Cards = State_foud two equal cards   / 1 Card = No Pairs, card given is random card without a class
            Card_ids=decide_action();
            if (Card_ids[1]==0){
                //send voice comand to turn both card
                //send
            }
            else{

            }
                //if YES turn them-->Prolog gives back action turnCards 1&2-->Prolog(Rule:act(Card1,Card2)-->turnBothCards)
                    //add them to card collection
                //if No turn 1 card with no class-->Prolog(Rule:act(Card1,_)-->FindOneUnknownCard)
                    //search for cards with no class-->Prolog-->give back 1 Card
                    // TurnCard 1--> Observe result
                        //if we have EQUAL Card in memory ,choose it and take it -->(Prolog:findEqualCard-->returns 2 cards-->turnPair)
                        //if NOT , choose antoher unknown card-->(Prolog: pickRandomCard(RandCard)) Prolog comparisson, check if by luck, they have the same class -->(Prolog:doIHavePair(False)-->turnRand)

            //If WON cards-->remove those instances from the knowledge -->Play again
            //if Not, go to next Player--> update MyTurn to False
        }else{
            std::cout<<"Oponent is playing"<<std::endl;
            //observe and store data
            //Observe the two cards choosen by player
            // If are equal, remove those instances from the knowledge
            //if not store them in memory
            //(optional(if we have time, probalby not): if not equal and disapear--> Cheating)

        }
        //Check for GameStatus (who is winning)--> Has he won?
    }



    //NodeManager nm;


    ros::spin();
}**/
