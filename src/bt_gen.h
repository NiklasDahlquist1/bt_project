
#ifndef BT_GEN_H
#define BT_GEN_H



#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf2/LinearMath/Quaternion.h>
#include <sstream>

#include <iostream>
#include <fstream>

#include <vector>
#include <tuple>


#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/blackboard.h"

#include "test_cpp/auction_winner.h"

//#include "actions.h"



namespace gen_bt 
{

    class Node_bt
    {
        private:
            std::string nodeName;
            std::vector<Node_bt*> childs;
            std::vector<std::tuple<std::string, std::string, std::string>> attributes; //(name, value, attributeType) attributetype used for generating model tree (can be ignored for ex. ID)
            Node_bt* parent;

            bool memoryAllocated = false; //keep track if this node was allocated with new, and thus should be deleted




        void indent(std::stringstream& fileStream, int depth)
            {
                for(int i = 0; i < depth; ++i)
                {
                    fileStream << "    "; //set how the indentation should be made (with 4 spaces in this case)
                }
            }


        public:



            //set to true if this node was created by its parent node
            void setMemAlloc(bool memAlloc)
            {
                this->memoryAllocated = memAlloc;
                return;
            }            



            //
            std::string getNodeName()
            {
                return this->nodeName;
            }
            
            std::vector<std::tuple<std::string, std::string, std::string>> getAttributes()
            {
                return this->attributes;
            }
            
            std::string getAttributebyName(std::string name)
            {
                for(std::tuple<std::string, std::string, std::string> attr : this->attributes)
                {
                    if(std::get<0>(attr) == name)
                    {
                        return std::get<1>(attr);
                    }
                }
                return "";
            }



            Node_bt(std::string nodeName, Node_bt* parent = nullptr)
            {
                this->nodeName = nodeName;
                this->parent = parent;
            }
            
            ~Node_bt()
            {
                for( Node_bt* c : this->childs)
                {
                    if(c->memoryAllocated == true)
                    {
                        delete c;
                        //std::cout << "Destroyed" << std::endl;
                    }
                }
            }

            void setParent(Node_bt* parent)
            {
                this->parent = parent;
            }

            void addChild(Node_bt& child)
            {
                //printf("%p\n", &child);
                child.setParent(this);
                this->childs.push_back(&child);
            }

            Node_bt* addAttribute(std::string argument, std::string value, std::string attributeType)
            {
                auto t = std::make_tuple(argument, value, attributeType);
                attributes.push_back(t);

                return this;
            }

            Node_bt* createChild(std::string name)
            {
                Node_bt* newChild = new Node_bt(name, this);
                this->childs.push_back(newChild);

                newChild->setMemAlloc(true);
                return newChild;
            }

            void replaceChild(Node_bt& oldChild, Node_bt& newChild) 
            {
                bool replaced = false;
                int i = 0;
                for(Node_bt* c : this->childs)
                {
                    if(&oldChild == c)
                    {
                        newChild.setParent(this);
                        childs[i] = &newChild;
                        replaced = true;
                    }
                    i++;
                }

                if(replaced == false)
                {
                    printf("Nothing was replaces, node did not exist?\n");
                }
            }

            void replaceNode(Node_bt& newNode)
            {
                if(parent != nullptr)
                {
                    parent->replaceChild(*this, newNode);
                }
                else
                {
                    std::cout << "NULL pointer in replaceNode" << std::endl;
                    throw std::invalid_argument("null pointer");
                }
            }

            std::vector<Node_bt*> findChild(std::string name)
            {
                std::vector<Node_bt*> ret;

                //loop through all childs and return object with the right name
                for(Node_bt* child : this->childs)
                {
                    if( child->nodeName == name)
                    {
                        ret.push_back(child);
                    }
                    
                    std::vector<Node_bt*> foundGrandChildren = child->findChild(name);
                    int size = foundGrandChildren.size();
                    if(size > 0)
                    {
                        //printf("%i", size);
                        for(int i = 0; i < size; ++size)
                        {
                            ret.push_back(foundGrandChildren[i]);
                        }
                    }
                }

                return ret;
            }

            std::vector<Node_bt*> getAllChilds(std::string name)
            {
                std::vector<Node_bt*> nodes;
                for(Node_bt* n : this->childs)
                {
                     //TODO, fix this for cleaning up memory from new()
                }
            }


            std::vector<std::string> findAllNames()
            {
                std::vector<std::string> names;

                names.push_back(this->nodeName);


                for(Node_bt* child : this->childs)
                {
                    //check this childs children
                    std::vector<std::string> moreNames = child->findAllNames();
                    //also check this child
                    moreNames.push_back(child->nodeName);

                    for(std::string nameCandidate : moreNames)
                    {
                        bool nameUnique = true;
                        for(std::string name : names)
                        {
                            if(name == nameCandidate)
                            {
                                nameUnique = false;
                            }
                        }
                        if(nameUnique)
                        {
                            names.push_back(nameCandidate);
                        }
                    }
                }
                return names;
            }

            std::vector<Node_bt*> findUniqueNodes() //maybe workS?
            {
                std::vector<Node_bt*> nodes;

                nodes.push_back(this);

                for(Node_bt* child : this->childs)
                {
                    //check this childs children
                    std::vector<Node_bt*> moreNodes = child->findUniqueNodes();

                    //also check this child
                    moreNodes.push_back(child);

                    for(Node_bt* nodeCandidate : moreNodes)
                    {
                        bool nodeUnique = true;
                        for(Node_bt* currentNode : nodes)
                        {
                            if(nodeCandidate->nodeName == currentNode->nodeName)
                            {
                                nodeUnique = false;
                            }
                        }
                        if(nodeUnique)
                        {
                            nodes.push_back(nodeCandidate);
                        }
                    }
                }
                return nodes;
            }

            std::vector<Node_bt*> findUniqueNodesBasedOnID() //maybe workS?
            {
                std::vector<Node_bt*> nodes;

                if(this->getAttributebyName("ID") != "") //node has no "ID", it should not be included
                {
                    nodes.push_back(this);
                }
                

                for(Node_bt* child : this->childs)
                {
                    //check this childs children
                    std::vector<Node_bt*> moreNodes = child->findUniqueNodesBasedOnID();

                    //also check this child
                    moreNodes.push_back(child);

                    for(Node_bt* nodeCandidate : moreNodes)
                    {
                        bool nodeUnique = true;
                        for(Node_bt* currentNode : nodes)
                        {
                            if(nodeCandidate->getAttributebyName("ID") == "") //node has no "ID", it should not be included
                            {
                                nodeUnique = false;
                                continue;
                            }
                            else if(nodeCandidate->getAttributebyName("ID") == currentNode->getAttributebyName("ID"))
                            {
                                nodeUnique = false;
                                continue;
                            }
                        }
                        if(nodeUnique)
                        {
                            nodes.push_back(nodeCandidate);
                        }
                    }
                }
                return nodes;
            }
            
            // print in xml format
            void printTo(std::stringstream& fileStream, int depth=0)
            {
                indent(fileStream, depth);

                fileStream << "<" << this->nodeName;
                for(const std::tuple<std::string, std::string, std::string> attr : attributes)
                {
                    fileStream << " " << std::get<0>(attr) << "=\"" << std::get<1>(attr) << "\"";
                }
                fileStream << ">" << std::endl;

                for(Node_bt* child : childs)
                {
                    child->printTo(fileStream, depth + 1);
                }

                indent(fileStream, depth);
                fileStream << "</" << this->nodeName << ">" << std::endl;
            }


            void printName()
            {
                std::cout << this->nodeName << " is my name" << std::endl;
            }



            void addInSequence(Node_bt* nodeToAdd)
            {
                if(this->parent == nullptr)
                {
                    std::cout << "Parent does not exist, cannot replace and add in sequence." << std::endl;
                }

                Node_bt* sequenceNode = new Node_bt("ReactiveSequence");
                sequenceNode->setMemAlloc(true);

                this->replaceNode(*sequenceNode);
                

                sequenceNode->addChild(*nodeToAdd);

                sequenceNode->addChild(*this);

                //create sequence node
                //replace this node with sequence node
                //add this to sequence node
                //add nodeToAdd to sequence
                //game?
            }


            void addInFallback(Node_bt* nodeToAdd)
            {
                if(this->parent == nullptr)
                {
                    std::cout << "Parent does not exist, cannot replace and add in fallback." << std::endl;
                }

                Node_bt* fallbackNode = new Node_bt("ReactiveFallback");
                fallbackNode->setMemAlloc(true);

                this->replaceNode(*fallbackNode);
                fallbackNode->addChild(*this);

                fallbackNode->addChild(*nodeToAdd);

            }
    };



// functions


    std::vector<std::string> splitString (std::string s, std::string delimiter) 
    {
        size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        std::string token;
        std::vector<std::string> res;

        while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
            token = s.substr (pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;
            res.push_back (token);
        }

        res.push_back (s.substr (pos_start));
        return res;
    }




    // print the models at the end of the xml
    void printModelGraph(Node_bt& root, std::stringstream& stream)
    {
        std::vector<Node_bt*> nodes = root.findUniqueNodesBasedOnID();

        Node_bt treeModels("TreeNodesModel");

        Node_bt* nodeptr;
        for(Node_bt* node : nodes)
        {
            nodeptr = treeModels.createChild(node->getNodeName());

            for(std::tuple<std::string, std::string, std::string> attr : node->getAttributes())
            {
                if(std::get<2>(attr) == "")
                {
                    nodeptr->addAttribute(std::get<0>(attr), std::get<1>(attr), "");
                }
                else if (std::get<2>(attr) == "output_port" || std::get<2>(attr) == "input_port")
                {
                    nodeptr->createChild(std::get<2>(attr))->addAttribute("name", std::get<0>(attr), "");
                }
            }
        }
        treeModels.printTo(stream);
    }



    struct node_arg
    {
        std::string name;
        std::string arg;
        std::string type;
    };

    struct behavior_test
    {
        std::string name;
        std::string preCondition;
        std::string effect;

        std::vector<node_arg> bt_arguments; //used for generating nodes for each name
    };

    



    const std::vector<behavior_test> available_behaviors = { {"HasCollisionFreeWP", "", "", {{"ID", "HasCollisionFreeWP", ""}}},
                                                       {"UAVAtPoint", "", "", {{"ID", "UAVAtPoint", ""}, {"point", "0;0;0", "input_port"} }},
                                                       {"IsFlying", "", "", {{"ID", "IsFlying", ""}}},
                                                       {"TakeOff", "", "IsFlying", {{"ID", "TakeOff", ""}}},
                                                       {"GetPointFromPotentialField", "IsFlying", "HasCollisionFreeWP", {{"ID", "GetPointFromPotentialField", ""}, {"goalPose", "{goalPose}", "output_port"} }},
                                                       {"PublishUAVWayPoint", "HasCollisionFreeWP", "UAVAtPoint", {{"ID", "PublishUAVWayPoint", ""}, {"goalPose", "{goalPose}", "input_port"}}}, 
                                                       
                                                       
                                                       {"UAVAtHomePoint", "", "", {{"ID", "PublishHomePose", ""}}}, //
                                                       {"PublishHomePose", "", "", {{"ID", "PublishHomePose", ""}}}, //

                                                       {"Empty", "", "", {{"ID", "Empty", ""}}}, //

                                                       {"DelayOnce", "", "", {{"ID", "DelayOnce", ""}, {"delayTime", "8", "input_port"}}}, 
                                                     };




    //return true if a behavior was found, and stores it in behavior
    bool findBehaviorFromName(behavior_test& behavior, const std::string& name, const std::vector<behavior_test>& available_behaviors)
    {
        bool success = false;
        for(const behavior_test& b : available_behaviors)
        {
            if(b.name == name)
            {
                success = true;
                behavior = b;
                return success;
            }
        }
        return success;
    }

    //return true if a behavior was found, and stores it in behavior
    bool findBehaviorFromCondition(behavior_test& behavior, const std::string& condition, const std::vector<behavior_test>& available_behaviors)
    {
        bool success = false;
        for(const behavior_test& b : available_behaviors)
        {
            if(b.preCondition == condition)
            {
                success = true;
                behavior = b;
                return success;
            }
        }
        return success;
    }

    //return true if a behavior was found, and stores it in behavior
    bool findBehaviorFromEffect(behavior_test& behavior, const std::string& effect, const std::vector<behavior_test>& available_behaviors)
    {
        bool success = false;
        for(const behavior_test& b : available_behaviors)
        {
            if(b.effect == effect)
            {
                success = true;
                behavior = b;
                return success;
            }
        }
        return success;
    }

    Node_bt* createNodeFromBehavior(const behavior_test& behavior)
    {
        Node_bt* nodePtr = new Node_bt("Action");
        nodePtr->setMemAlloc(true); // to realease memory later

        for(node_arg attr : behavior.bt_arguments)
        {
            nodePtr->addAttribute(attr.name, attr.arg, attr.type);
        }
        return nodePtr;
    }

    void gen(const std::vector<behavior_test>& available_behaviors, const std::string& goalBehaviorEffect, const std::string& taskData, std::stringstream& stream)
    {
        behavior_test goalAction;
        behavior_test goalCondition;
        if(!findBehaviorFromName(goalCondition, goalBehaviorEffect, available_behaviors)) 
            std::cout << "FAILED, could not find goal condition" << std::endl;
        if(!findBehaviorFromEffect(goalAction, goalBehaviorEffect, available_behaviors)) 
            std::cout << "FAILED, could not find goal action" << std::endl;


        //std::cout << "goal condition: " << goalCondition.name << std::endl;
        //std::cout << "goal action: " << goalAction.name << std::endl;
        

        Node_bt root("BehaviorTree");
        root.addAttribute("ID", "BehaviorTree", "");
        

        std::vector<Node_bt*> conditionsToDo;
        std::vector<Node_bt*> actionsToDo;


        // TODO, add standard part of BT. safety checks etc.
        Node_bt* seq = root.createChild("ReactiveSequence");
        seq->createChild("Action")->addAttribute("ID", "SafetyChecks", ""); 



        Node_bt* startOfTaskBT = seq;


        //add custom first condition and then start expanding
        // depending on task, add a check if the task cannot be completed and return failure in that case
        if(goalBehaviorEffect == "UAVAtPoint")
        {
            startOfTaskBT->createChild("Action")->addAttribute("ID", "CheckTaskFailUAVAtPoint", ""); 

            Node_bt* nodePtr = startOfTaskBT->createChild("Action");
            nodePtr->addAttribute("ID", goalCondition.name, "");
            nodePtr->addAttribute("point", taskData, "input_port");
            conditionsToDo.push_back(nodePtr);
        }
        else
        {
            // nothing special to add to first condition
            Node_bt* nodePtr = startOfTaskBT->createChild("Action");
            nodePtr->addAttribute("ID", goalCondition.name, "");
            conditionsToDo.push_back(nodePtr);
        }

        bool doneGenerating = false;

        while(!doneGenerating)
        {
            for(Node_bt* n : conditionsToDo)
            {
                behavior_test behavior;
                if(!findBehaviorFromEffect(behavior, n->getAttributebyName("ID"), available_behaviors))
                    std::cout << "Could not find action for that condition" << std::endl;

                if(behavior.name == "") //no condition to add
                    continue;

               Node_bt* newAction = createNodeFromBehavior(behavior);


                //expand with fallback
                n->addInFallback(newAction);

                //add node to actions to do
                actionsToDo.push_back(newAction);
            }
            conditionsToDo.erase(conditionsToDo.begin(), conditionsToDo.end());


            for(Node_bt* n : actionsToDo)
            {
                behavior_test behaviorAction;
                behavior_test behaviorCondition;
                findBehaviorFromName(behaviorAction, n->getAttributebyName("ID"), available_behaviors);
                findBehaviorFromName(behaviorCondition, behaviorAction.preCondition, available_behaviors);

                if(behaviorCondition.name == "") //no condition to add
                    continue;

                Node_bt* newCondition = createNodeFromBehavior(behaviorCondition);

                //expand with sequence
                n->addInSequence(newCondition);

                //add node to actions to do
                conditionsToDo.push_back(newCondition);
            }
            actionsToDo.erase(actionsToDo.begin(), actionsToDo.end());

            if(conditionsToDo.size() == 0 && actionsToDo.size() == 0)
            {
                doneGenerating = true;
            }
        }
        

        
        stream << "<?xml version=\"1.0\"?>" << std::endl << "<!-- ////////// -->" << std::endl; 
        stream << "<root main_tree_to_execute=\"BehaviorTree\">" << std::endl;

        root.printTo(stream); //print the behavior tree
        stream << "<!-- ////////// -->" << std::endl;
        printModelGraph(root, stream); // print the models for the behaviors
        stream << "<!-- ////////// -->" << std::endl;

        stream << "</root>";


        //std::cout << stream.str() << std::endl; //print now for testing
    }

    std::string getXMLBasedOnAuction(const std::vector<behavior_test>& available_behaviors, const test_cpp::auction_winner task)
    {
        std::stringstream stream;

        //behavior_test goalAction;
        //behavior_test goalCondition;
        //if(!findBehaviorFromName(goalCondition, goalBehaviorEffect, available_behaviors)) 
        //    std::cout << "FAILED, could not find goal condition" << std::endl;
        //if(!findBehaviorFromEffect(goalAction, goalBehaviorEffect, available_behaviors)) 
        //    std::cout << "FAILED, could not find goal action" << std::endl;


        //std::cout << "goal condition: " << goalCondition.name << std::endl;
        //std::cout << "goal action: " << goalAction.name << std::endl;
        

        Node_bt root("BehaviorTree");
        root.addAttribute("ID", "BehaviorTree", "");
        

        std::vector<Node_bt*> conditionsToDo;
        std::vector<Node_bt*> actionsToDo;


        // TODO, add standard part of BT. safety checks etc.
        Node_bt* seq = root.createChild("ReactiveSequence");
        seq->createChild("Action")->addAttribute("ID", "SafetyChecks", ""); 


        Node_bt* startOfTaskBT = seq;




        //add custom first condition and then start expanding
        // depending on task, add a check if the task cannot be completed and return failure in that case
        if(task.task_name == "moveTo")
        {
            auto parts = splitString(task.task_data, ";");

            startOfTaskBT->createChild("Action")->addAttribute("ID", "CheckTaskFailUAVAtPoint", ""); //add condition to check if the current task fails

            Node_bt* nodePtr = startOfTaskBT->createChild("Action");
            nodePtr->addAttribute("ID", "UAVAtPoint", "");
            nodePtr->addAttribute("point", parts[0] + ";" + parts[1] + ";" + parts[2], "input_port");
            conditionsToDo.push_back(nodePtr); //
        }
        else if (task.task_name == "noTaskBT")
        {
            auto parts = splitString(task.task_data, ";");

            /*behavior_test delay;
            findBehaviorFromName(delay, "delayOnce", available_behaviors);
            Node_bt* delayNode = createNodeFromBehavior(delay);
            startOfTaskBT->addChild(delayNode);
            */

            startOfTaskBT->createChild("Action")->addAttribute("ID", "DelayOnce", "")->addAttribute("delayTime", "10", "input_port"); //TODO remove

            startOfTaskBT->createChild("Action")->addAttribute("ID", "CheckTaskFailUAVAtPoint", ""); //add condition to check if the current task fails

            Node_bt* nodePtr = startOfTaskBT->createChild("Action");
            nodePtr->addAttribute("ID", "UAVAtPoint", "");
            nodePtr->addAttribute("point", parts[0] + ";" + parts[1] + ";" + parts[2], "input_port");
            conditionsToDo.push_back(nodePtr); //
        }
        
        else
        {
            //no know task, maybe bad? for now, return some standard BT

            // nothing special to add to first condition
            Node_bt* nodePtr = startOfTaskBT->createChild("Action");
            nodePtr->addAttribute("ID", "Empty", "");
            conditionsToDo.push_back(nodePtr);
        }

        bool doneGenerating = false;

        while(!doneGenerating)
        {
            for(Node_bt* n : conditionsToDo)
            {
                behavior_test behavior;
                if(!findBehaviorFromEffect(behavior, n->getAttributebyName("ID"), available_behaviors))
                    std::cout << "Could not find action for that condition" << std::endl;

                if(behavior.name == "") //no condition to add
                    ROS_INFO_STREAM("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa");
                    //continue;

               Node_bt* newAction = createNodeFromBehavior(behavior);


                //expand with fallback
                n->addInFallback(newAction);

                //add node to actions to do
                actionsToDo.push_back(newAction);
            }
            conditionsToDo.erase(conditionsToDo.begin(), conditionsToDo.end());


            for(Node_bt* n : actionsToDo)
            {
                behavior_test behaviorAction;
                behavior_test behaviorCondition;
                findBehaviorFromName(behaviorAction, n->getAttributebyName("ID"), available_behaviors);
                findBehaviorFromName(behaviorCondition, behaviorAction.preCondition, available_behaviors);

                if(behaviorCondition.name == "") //no condition to add
                    continue;

                Node_bt* newCondition = createNodeFromBehavior(behaviorCondition);

                //expand with sequence
                n->addInSequence(newCondition);

                //add node to actions to do
                conditionsToDo.push_back(newCondition);
            }
            actionsToDo.erase(actionsToDo.begin(), actionsToDo.end());

            if(conditionsToDo.size() == 0 && actionsToDo.size() == 0)
            {
                doneGenerating = true;
            }
        }

        
        stream << "<?xml version=\"1.0\"?>" << std::endl << "<!-- ////////// -->" << std::endl; 
        stream << "<root main_tree_to_execute=\"BehaviorTree\">" << std::endl;

        root.printTo(stream); //print the behavior tree
        stream << "<!-- ////////// -->" << std::endl;
        printModelGraph(root, stream); // print the models for the behaviors
        stream << "<!-- ////////// -->" << std::endl;

        stream << "</root>";


        //std::cout << stream.str() << std::endl; //print now for testing

        return stream.str();
    }


    // generate BT based on task name
    std::string generateXMLForBT(const std::string task, const std::string taskData)
    {
        std::stringstream stream;

        if(task == "noTask")
        {
            // return to home position
            //gen(available_behaviors, "PublishHomePose", "", stream);
            auto parts = splitString(taskData, ";");
            gen(available_behaviors, "UAVAtPoint", parts[0] + ";" + parts[1] + ";" + parts[2], stream); // send x,y,z


            //std::cout << stream.str() << std::endl; // print entire xml
        }

        if(task == "moveTo")
        {
            auto parts = splitString(taskData, ";");
            gen(available_behaviors, "UAVAtPoint", parts[0] + ";" + parts[1] + ";" + parts[2], stream); // send x,y,z


            //std::cout << stream.str() << std::endl; // print entire xml
        }
        else
        {
            //fallback BT, task cannot be completed. It should be sent back to auctioning server from calling function TODO
            // or is this an error?
        }

        return stream.str();
    }


    void loadNewBT(BT::Tree& btTree, BT::BehaviorTreeFactory& factory, const test_cpp::auction_winner task)
    {
        //TODO, maybe allways call xml function? maybe pass NodeBT to the generation function and just add the task aprt
        if(task.task_name == "moveTo")
        {
            std::string bt = getXMLBasedOnAuction(available_behaviors, task);
            btTree = factory.createTreeFromText(bt);
        }
        else if(task.task_name == "noTaskBT")
        {
            std::string bt = getXMLBasedOnAuction(available_behaviors, task);
            btTree = factory.createTreeFromText(bt);
        }
        else
        {
            ROS_INFO_STREAM("Can not do task: " << task.task_name);
            //Send task back to auctioning server TODO
        }

        /*
        if(task.task_name == "moveTo")
        {
            std::string bt = generateXMLForBT("moveTo", task.task_data);
            btTree = factory.createTreeFromText(bt);
        }
        else
        {
            ROS_INFO_STREAM("Can not do task: " << task.task_name);
            //Send task back to auctioning server TODO
        }*/
        return;
    }

    
    
} // namespace gen_bt





namespace bt
{

} // end namespace bt




#endif

