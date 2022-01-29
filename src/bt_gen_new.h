



#ifndef BT_GEN_NEW_H
#define BT_GEN_NEW_H


#include "ros/package.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include <tuple>

#include "bt_project/auction_winner.h"


namespace bt
{

    struct node_arg
    {
        std::string name;
        std::string arg;
        std::string type;
    };


    struct behavior_test
    {
        std::string name;
        std::vector<std::string> preConditions;
        std::vector<std::string> effects;

        std::vector<node_arg> bt_arguments; //used for generating nodes for each name
    };

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

            Node_bt* getParent()
            {
                return this->parent;
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

            void addChild(Node_bt* child)
            {
                //printf("%p\n", &child);
                child->setParent(this);
                this->childs.push_back(child);
            }

            void addChildBeforeLast(Node_bt* child)
            {
                int size = this->childs.size();
                if(size == 0)
                    std::cout << "ERROR, addChildBeforeLast. Has ni childs, cannot add before last child" << std::endl;
                //printf("%p\n", &child);
                child->setParent(this);
                this->childs.push_back(childs[size - 1]); // save current last child 
                this->childs[size - 1] = child; // save child at the position of the previous last child
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
                     //maybe not needed anymore, remove?
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
                

                sequenceNode->addChild(nodeToAdd);

                sequenceNode->addChild(this);

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
                fallbackNode->addChild(this);

                fallbackNode->addChild(nodeToAdd);

            }
    };

    // functions




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
        for(const behavior_test& b : available_behaviors) // loop through behaviors
        {
            for(const std::string& s : b.preConditions) // loop through conditions
            {
                if(s == condition)
                {
                    success = true;
                    behavior = b;
                    return success;
                }
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
            for(const std::string& s : b.effects) // loop through effects
            {
                if(s == effect)
                {
                    success = true;
                    behavior = b;
                    return success;
                }
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



    Node_bt* getStartCondition(Node_bt* root, bt_project::auction_winner task, const std::vector<behavior_test>& available_behaviors)
    {
        //Node_bt root("BehaviorTree");
        //root.addAttribute("ID", "BehaviorTree", "");


        Node_bt* seq = root->createChild("ReactiveSequence");

        // safety checks
        behavior_test safetyBehavior;
        findBehaviorFromName(safetyBehavior, "SafetyChecks", available_behaviors);
        seq->addChild(createNodeFromBehavior(safetyBehavior));
        Node_bt* startOfTaskBT = seq;


        Node_bt* startCondition;
        //custom first condition and then start expanding
        if(task.task_name == "explore")
        {
            behavior_test checkTaskFailBehavior;
            findBehaviorFromName(checkTaskFailBehavior, "CheckTaskFailExplore", available_behaviors);
            startOfTaskBT->addChild(createNodeFromBehavior(checkTaskFailBehavior));

            auto parts = splitString(task.task_data, ";");
            //TODO, check


            startCondition = startOfTaskBT->createChild("Action");
            startCondition->addAttribute("ID", "ExploredSomeTime", "");
            startCondition->addAttribute("startPoint", parts[0] + ";" + parts[1] + ";" + parts[2], "input_port"); 
            startCondition->addAttribute("exploreTime", parts[3], "input_port");
            startCondition->addAttribute("goalPoint", "{point}", "output_port");
        }
        else if(task.task_name == "moveTo")
        {
            behavior_test checkTaskFailBehavior;
            findBehaviorFromName(checkTaskFailBehavior, "CheckTaskFailUAVAtPoint", available_behaviors);
            startOfTaskBT->addChild(createNodeFromBehavior(checkTaskFailBehavior));

            auto parts = splitString(task.task_data, ";");
            //TODO, check


            startCondition = startOfTaskBT->createChild("Action");
            startCondition->addAttribute("ID", "UAVAtPoint", "");
            startCondition->addAttribute("point", parts[0] + ";" + parts[1] + ";" + parts[2], "input_port"); 
            startCondition->addAttribute("goalPoint", "{goalPoint}", "output_port"); 
        }
        else if(task.task_name == "land")
        {
            behavior_test delay;
            findBehaviorFromName(delay, "DelayOnce", available_behaviors); //delay with standard delay time
            startOfTaskBT->addChild(createNodeFromBehavior(delay));

            auto parts = splitString(task.task_data, ";");
            //TODO, check


            startCondition = startOfTaskBT->createChild("Action");
            //startCondition->addAttribute("ID", "UAVAtPoint", "");
            startCondition->addAttribute("ID", "LandedAtPoint", "");
            startCondition->addAttribute("pointToLandAt", parts[0] + ";" + parts[1] + ";" + parts[2], "input_port"); 
            startCondition->addAttribute("pointAboveLanding", "{point}", "output_port"); 
            startCondition->addAttribute("landingPoint", "{homePoint}", "output_port"); 
        }
        else if(task.task_name == "noTaskBT")
        {
            behavior_test delay;
            findBehaviorFromName(delay, "DelayOnce", available_behaviors); //delay with standard delay time
            startOfTaskBT->addChild(createNodeFromBehavior(delay));

            auto parts = splitString(task.task_data, ";");
            //TODO, check


            startCondition = startOfTaskBT->createChild("Action");
            startCondition->addAttribute("ID", "UAVAtPoint", "");
            startCondition->addAttribute("point", parts[0] + ";" + parts[1] + ";" + parts[2], "input_port"); 
            startCondition->addAttribute("goalPoint", "{goalPoint}", "output_port"); 
        }
        else
        {
            startCondition = startOfTaskBT->createChild("Empty")->addAttribute("ID", "Empty", "");
        }

        return startCondition;

    }





    // generates bt graph based on startCondition, condition can not be a "root node"
    void generateTreeGraph(Node_bt* startCondition, const std::vector<behavior_test>& available_behaviors)
    {

        //behavior_test goalAction;
        //behavior_test goalCondition;
        //if(!findBehaviorFromName(goalCondition, goalBehaviorEffect, available_behaviors)) 
        //    std::cout << "FAILED, could not find goal condition" << std::endl;
        //if(!findBehaviorFromEffect(goalAction, goalBehaviorEffect, available_behaviors)) 
        //    std::cout << "FAILED, could not find goal action" << std::endl;


        //std::cout << "goal condition: " << goalCondition.name << std::endl;
        //std::cout << "goal action: " << goalAction.name << std::endl;
        

       

        std::vector<Node_bt*> conditionsToDo;
        std::vector<Node_bt*> actionsToDo;

        conditionsToDo.push_back(startCondition);

        //std::cout << startCondition->getNodeName() << " " << startCondition->getAttributebyName("ID") <<  std::endl;

        bool doneGenerating = false;

        while(!doneGenerating)
        {
            for(Node_bt* n : conditionsToDo)
            {
                behavior_test behavior;
                if(!findBehaviorFromEffect(behavior, n->getAttributebyName("ID"), available_behaviors))
                    std::cout << "Could not find action for that condition: " << n->getAttributebyName("ID") << std::endl;

                if(behavior.name == "") //no condition to add
                    std::cout << "AAAAAAAAAAAAAAaa" << std::endl;
                    //ROS_INFO_STREAM("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa");
                    //continue; //TODO ?????

               Node_bt* newAction = createNodeFromBehavior(behavior);
               //std::cout << behavior.name << std::endl;


                //expand with fallback
                n->addInFallback(newAction);

                //add node to actions to do
                actionsToDo.push_back(newAction);
            }
            conditionsToDo.erase(conditionsToDo.begin(), conditionsToDo.end());


            for(Node_bt* n : actionsToDo)
            {
                bool firstAdd = true; // TODO, make less hacky
                behavior_test behaviorAction;
                findBehaviorFromName(behaviorAction, n->getAttributebyName("ID"), available_behaviors);
                for(const std::string& preCondition : behaviorAction.preConditions) // expand for each preCondition
                {
                    behavior_test behaviorCondition;
                    if(!findBehaviorFromName(behaviorCondition, preCondition, available_behaviors))
                        std::cout << "WARNING, did not find action to complete preCondition " << preCondition << std::endl;

                    if(behaviorCondition.name == "") //no condition to add
                        continue;

                    Node_bt* newCondition = createNodeFromBehavior(behaviorCondition);

                    if(firstAdd)
                    {
                        //expand with sequence
                        n->addInSequence(newCondition);
                        firstAdd = false;
                    }
                    else
                    {
                        n->getParent()->addChildBeforeLast(newCondition);
                    }


                    //add node to actions to do
                    conditionsToDo.push_back(newCondition);
                }

            }
            actionsToDo.erase(actionsToDo.begin(), actionsToDo.end());

            if(conditionsToDo.size() == 0 && actionsToDo.size() == 0)
            {
                doneGenerating = true;
            }
        }

/*        std::stringstream stream;

        stream << "<?xml version=\"1.0\"?>" << std::endl << "<!-- ////////// -->" << std::endl; 
        stream << "<root main_tree_to_execute=\"BehaviorTree\">" << std::endl;

        root.printTo(stream); //print the behavior tree
        stream << "<!-- ////////// -->" << std::endl;
        printModelGraph(root, stream); // print the models for the behaviors
        stream << "<!-- ////////// -->" << std::endl;

        stream << "</root>";

*/
        return;
        //std::cout << stream.str() << std::endl; //print now for testing

        //return stream.str();
    }

    std::string getXMLFromGraph(Node_bt* root)
    {
        std::stringstream stream;

        stream << "<?xml version=\"1.0\"?>" << std::endl << "<!-- ////////// -->" << std::endl; 
        stream << "<root main_tree_to_execute=\"BehaviorTree\">" << std::endl;

        root->printTo(stream); //print the behavior tree
        stream << "<!-- ////////// -->" << std::endl;
        printModelGraph(*root, stream); // print the models for the behaviors
        stream << "<!-- ////////// -->" << std::endl;

        stream << "</root>";

        return stream.str();
    }

std::string remove(bt_project::auction_winner task, const std::vector<behavior_test>& available_behaviors)
    {

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


        Node_bt* seq = root.createChild("ReactiveSequence");

        // safety checks
        behavior_test safetyBehavior;
        findBehaviorFromName(safetyBehavior, "SafetyChecks", available_behaviors);
        seq->addChild(createNodeFromBehavior(safetyBehavior));
        Node_bt* startOfTaskBT = seq;


        Node_bt* startCondition;
        //custom first condition and then start expanding
        if(task.task_name == "explore")
        {
            behavior_test checkTaskFailBehavior;
            findBehaviorFromName(checkTaskFailBehavior, "CheckTaskFailExplore", available_behaviors);
            startOfTaskBT->addChild(createNodeFromBehavior(checkTaskFailBehavior));

            auto parts = splitString(task.task_data, ";");
            //TODO, check


            startCondition = startOfTaskBT->createChild("Action");
            startCondition->addAttribute("ID", "ExploredSomeTime", "");
            startCondition->addAttribute("goalPoint", parts[0] + ";" + parts[1] + ";" + parts[2], "input_port"); 
            startCondition->addAttribute("exploreTime", parts[3], "input_port"); 
        }
        else if(task.task_name == "moveTo")
        {
            behavior_test checkTaskFailBehavior;
            findBehaviorFromName(checkTaskFailBehavior, "CheckTaskFailUAVAtPoint", available_behaviors);
            startOfTaskBT->addChild(createNodeFromBehavior(checkTaskFailBehavior));

            auto parts = splitString(task.task_data, ";");
            //TODO, check


            startCondition = startOfTaskBT->createChild("Action");
            startCondition->addAttribute("ID", "UAVAtPoint", "");
            startCondition->addAttribute("point", parts[0] + ";" + parts[1] + ";" + parts[2], "input_port"); 
        }
        else
        {
            startCondition = startOfTaskBT->createChild("Empty")->addAttribute("ID", "Empty", "");
        }

       

        std::vector<Node_bt*> conditionsToDo;
        std::vector<Node_bt*> actionsToDo;

        actionsToDo.push_back(startCondition);

        //std::cout << startCondition->getNodeName() << " " << startCondition->getAttributebyName("ID") <<  std::endl;

        bool doneGenerating = false;

        while(!doneGenerating)
        {
            for(Node_bt* n : conditionsToDo)
            {
                behavior_test behavior;
                if(!findBehaviorFromEffect(behavior, n->getAttributebyName("ID"), available_behaviors))
                    std::cout << "Could not find action for that condition: " << n->getAttributebyName("ID") << std::endl;

                if(behavior.name == "") //no condition to add
                    std::cout << "AAAAAAAAAAAAAAaa" << std::endl;
                    //ROS_INFO_STREAM("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa");
                    //continue; //TODO ?????

               Node_bt* newAction = createNodeFromBehavior(behavior);
               std::cout << behavior.name << std::endl;


                //expand with fallback
                n->addInFallback(newAction);

                //add node to actions to do
                actionsToDo.push_back(newAction);
            }
            conditionsToDo.erase(conditionsToDo.begin(), conditionsToDo.end());


            for(Node_bt* n : actionsToDo)
            {
                bool firstAdd = true; // TODO, make less hacky
                behavior_test behaviorAction;
                findBehaviorFromName(behaviorAction, n->getAttributebyName("ID"), available_behaviors);
                for(const std::string& preCondition : behaviorAction.preConditions) // expand for each preCondition
                {
                    behavior_test behaviorCondition;
                    findBehaviorFromName(behaviorCondition, preCondition, available_behaviors);

                    if(behaviorCondition.name == "") //no condition to add
                        continue;

                    Node_bt* newCondition = createNodeFromBehavior(behaviorCondition);

                    if(firstAdd)
                    {
                        //expand with sequence
                        n->addInSequence(newCondition);
                        firstAdd = false;
                    }
                    else
                    {
                        n->getParent()->addChildBeforeLast(newCondition);
                    }


                    //add node to actions to do
                    conditionsToDo.push_back(newCondition);
                }

            }
            actionsToDo.erase(actionsToDo.begin(), actionsToDo.end());

            if(conditionsToDo.size() == 0 && actionsToDo.size() == 0)
            {
                doneGenerating = true;
            }
        }

        std::stringstream stream;

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



    std::vector<behavior_test> readBehaviorLibraryCSV(std::string fileBehaviorsAvailable, std::string fileBehaviorDefinition)
    {
        std::vector<behavior_test> content;

	    std::string line, word;

        std::fstream myFile;
        myFile.open(fileBehaviorsAvailable, std::fstream::in);//std::fstream::out | std::fstream::in | std::fstream::trunc);   



        // read available behaviors
        if(myFile.is_open())
        {
            while(std::getline(myFile, line))
            {
                auto parts = splitString(line, ",");
                if (parts.size() != 3) //ignore row with wrong size (nice to skip empty rows)
                {
                    if(parts[0] != "") // assume empty row (for now?) maybe require a name?
                    {
                        std::cout << "Error, incorrect CSV row: " << fileBehaviorsAvailable << std::endl; //ROS_INFO_STREAM("Error, incorrect CSV: " << file);
                    }
                    continue; // skip current row
                }                
                
                behavior_test behavior;
                behavior.name = parts[0];

                std::vector<std::string> partsPreConditions = splitString(parts[1], ";");
                for(const std::string& s : partsPreConditions)
                {
                    behavior.preConditions.push_back(s);
                }

                std::vector<std::string> partsEffects = splitString(parts[2], ";");
                for(const std::string& s : partsEffects)
                {
                    behavior.effects.push_back(s);
                }

                content.push_back(behavior);
            }
            myFile.close();
        }
        else
        {
            std::cout<<"Could not open the file\n";
        }

        myFile.open(fileBehaviorDefinition, std::fstream::in);//std::fstream::out | std::fstream::in | std::fstream::trunc);   


        if(myFile.is_open())
        {
            while(std::getline(myFile, line))
            {
                auto parts = splitString(line, ",");
                if (parts.size() != 4) //ignore row with wrong size (nice to skip empty rows)
                {
                    if(parts[0] != "") // assume empty row (for now?) maybe require a name?
                    {
                        std::cout << "Error, incorrect CSV row: " << fileBehaviorDefinition << std::endl; //ROS_INFO_STREAM("Error, incorrect CSV: " << file); 
                    }
                    continue;
                }

                // maybe slow loop, but works for now
                for(behavior_test& b : content)
                {
                    if(parts[0] == b.name)
                    {
                        node_arg arg;
                        arg.name = parts[1];
                        arg.arg = parts[2];
                        arg.type = parts[3];
                        b.bt_arguments.push_back(arg);
                        continue;
                    }
                }
            }

            myFile.close();
        }
        else
        {
            std::cout<<"Could not open the file\n";
        }

        std::cout << "(files loaded) Behaviors available: " << fileBehaviorsAvailable << ", definitions: " << fileBehaviorDefinition << std::endl;


        for(int i = 0; i < content.size() ;i++)
        {
            std::cout << "behavior " << i << ": " << content[i].name << std::endl;// << ", " << content[i].preCondition << ", " << content[i].effect << std::endl;

            for(const std::string& s : content[i].preConditions)
                std::cout << "  Condition: " << s << std::endl;
            for(const std::string& s : content[i].effects)
                std::cout << "  Effect: " << s << std::endl;


            for(const node_arg& n : content[i].bt_arguments)
                std::cout << "    node args: " << n.name << ", " << n.arg << ", " << n.type << std::endl;
        }


        return content;
    }





    void getNewTree(BT::Tree& tree, BT::BehaviorTreeFactory& factory, const bt_project::auction_winner& task, const std::vector<bt::behavior_test>& behaviorLibrary)
    {
        Node_bt root = bt::Node_bt("BehaviorTree");
        root.addAttribute("ID", "BehaviorTree", "");


        Node_bt* startCondition = getStartCondition(&root, task, behaviorLibrary); // get start condition based on specific task, and link to root node
        generateTreeGraph(startCondition, behaviorLibrary); // expands tree from start condition
        std::string xmlBT = getXMLFromGraph(&root);
        //std::cout << xmlBT << std::endl;

        tree = factory.createTreeFromText(xmlBT);



        //save to file (for looking at the bt later), TODO remove
        std::fstream saveFileStream;
        //std::cout << ros::package::getPath("bt_project") + "/../.." + ros::this_node::getName()  << std::endl;
        saveFileStream.open(ros::package::getPath("bt_project") + "/../.." + /*ros::this_node::getName()*/ "/singleTaskTree.xml", std::fstream::out);//std::fstream::in std::fstream::out | std::fstream::in | std::fstream::trunc);  
        if(saveFileStream.is_open())
        {
            saveFileStream << xmlBT;
            saveFileStream.close();    
        }
        else std::cout << "Could not open text file" << std::endl;

        return;
    }



} // END namespace bt




#endif







