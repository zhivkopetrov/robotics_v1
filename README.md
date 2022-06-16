# robotics_v1

prooject structure

cmake_helpers (CMake utilities)
  |--utils (static lib)
        |--resource_utils (static lib)
                   |--sdl_utils (static lib)
                   |     |--manager_utils (static lib)
                   |         |     |--game_engine (static lib)
resource_builder---|         |                 |--ros2_game_engine (static lib)
(standalone tool)            |                                  \           
                             |                                   \          
   robo_common (static lib)--|                                    \         
          |                                                        \        
          |                                                         \       
          |    robo_collector_interfaces                             \      
          |    (ros2 package interface)                               \     
          |      |                  \                                  \    
          |      |                   \                                  \   
          |--robo_collector_common    \                                  \                   
          |  (static lib)       \      \                                  \                   
          |                      \-----------|                             \
          |                                  |                             |
          |                                  |--robo_collector_gui---------|
          |                                  |  (ros2 package)             |
          |                                  |                             |
          |                                  |--robo_collector_controller--|
          |                                    (ros2 package)              |
          |             	                 	                           |
          |             	                 	                           |                                  
          |    robo_miner_interfaces                                       |
          |    (ros2 package interface)                                    |       
          |      |                  \                                      |    
          |      |                   \                                     |  
          |--robo_miner_common        \                                    |                  
          |  (static lib)    \         \                                   |                  
          |                  \---------------|                             |
          |                                  |                             |
          |                                  |--robo_miner_gui-------------|
          |                                  |  (ros2 package)             |
          |                                  |--robo_miner_controller      |
          |                                     (future ros2 package)      |
          |             	                 	                           |                                  
          |    robo_cleaner_interfaces             	                 	   |       
          |    (ros2 package interface)            	                 	   |            	                 	         
          |      |                  \              	                 	   |      
          |      |                   \               	                   |    
          |--robo_cleaner_common      \               	                   |                    
             (static lib)       \      \               	                   |                    
                                 \-----------|         	                   |
                                             |         	                   |
                                             |--robo_cleaner_gui-----------|
                                             |  (ros2 package)             
                                             |                             
                                             |--robo_cleaner_controller    
                                                (future ros2 package)
                                  
                                  