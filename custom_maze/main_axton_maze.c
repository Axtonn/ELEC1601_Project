#include "stdio.h"
#include "stdlib.h"
#include "SDL.h"
#include "SDL2_gfxPrimitives.h"
#include "time.h"

#include "formulas.h"
#include "wall.h"
#include "robot.h"

int done = 0;


int main(int argc, char *argv[]) {
    SDL_Window *window;
    SDL_Renderer *renderer;

    if(SDL_Init(SDL_INIT_VIDEO) < 0){
        return 1;
    }

    window = SDL_CreateWindow("Robot Maze", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT, SDL_WINDOW_OPENGL);
    window = SDL_CreateWindow("Robot Maze", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT, SDL_WINDOW_OPENGL);
    renderer = SDL_CreateRenderer(window, -1, 0);

    struct Robot robot;
    struct Wall_collection *head = NULL;
    int front_left_sensor, front_right_sensor=0;
    clock_t start_time, end_time;
    int msec;

    // SETUP MAZE
    // You can create your own maze here. line of code is adding a wall.
    // You describe position of top left corner of wall (x, y), then width and height going down/to right
    // Relative positions are used (OVERALL_WINDOW_WIDTH and OVERALL_WINDOW_HEIGHT)
    // But you can use absolute positions. 10 is used as the width, but you can change this.
    //head, key, position horizontally ( addition = move right, subtraction = move left), position vertically( addition = movedown, subtraction = moveup), width(higher=thicker), length(higher = longer).
    insertAndSetFirstWall(&head, 1,  OVERALL_WINDOW_WIDTH/2, OVERALL_WINDOW_HEIGHT/2+50, 10, 200);// right wall below small
    insertAndSetFirstWall(&head, 2,  OVERALL_WINDOW_WIDTH/2-250, OVERALL_WINDOW_HEIGHT/2+100, 10, OVERALL_WINDOW_HEIGHT/2-10);// moist left bottomwall
    insertAndSetFirstWall(&head, 3,  OVERALL_WINDOW_WIDTH/2-250, OVERALL_WINDOW_HEIGHT/2+100, 150, 10);// hori line on top of robot
    insertAndSetFirstWall(&head, 4,  OVERALL_WINDOW_WIDTH/2-150, OVERALL_WINDOW_HEIGHT/2, 200, 10);// horizontal line aft first left turn
    insertAndSetFirstWall(&head, 5,  OVERALL_WINDOW_WIDTH/2-250, OVERALL_WINDOW_HEIGHT/2-200, 10, 300);//most left wall
    insertAndSetFirstWall(&head, 6,  OVERALL_WINDOW_WIDTH/2-150, OVERALL_WINDOW_HEIGHT/2-200, 10, 200);//vertical line at top left
    insertAndSetFirstWall(&head, 7,  OVERALL_WINDOW_WIDTH/2-250, OVERALL_WINDOW_HEIGHT/2-200, 600, 10);//top wall
    insertAndSetFirstWall(&head, 8,  OVERALL_WINDOW_WIDTH/2-50, OVERALL_WINDOW_HEIGHT/2-100, 150, 10);// hori line on top of key 4
    insertAndSetFirstWall(&head, 9,  OVERALL_WINDOW_WIDTH/2+200, OVERALL_WINDOW_HEIGHT/2-150, 10, 250);//most right wall long
    insertAndSetFirstWall(&head, 10,  OVERALL_WINDOW_WIDTH/2+100, OVERALL_WINDOW_HEIGHT/2-100, 10, 300);// long left wall oppisite to most right wall edit make hole
    insertAndSetFirstWall(&head, 11,  OVERALL_WINDOW_WIDTH/2+100, OVERALL_WINDOW_HEIGHT/2+200, OVERALL_WINDOW_WIDTH/2-100, 10);//btoom wall of exit
    insertAndSetFirstWall(&head, 12,  OVERALL_WINDOW_WIDTH/2+200, OVERALL_WINDOW_HEIGHT/2+100, OVERALL_WINDOW_WIDTH/2-100, 10);// topwall of exit
    insertAndSetFirstWall(&head, 13,  OVERALL_WINDOW_WIDTH/2-150, OVERALL_WINDOW_HEIGHT/2, 10, OVERALL_WINDOW_HEIGHT/2-200);//ver lin left side of narrow turn
    insertAndSetFirstWall(&head, 14,  OVERALL_WINDOW_WIDTH/2-250, OVERALL_WINDOW_HEIGHT/2+230, 250, 10);//bottom line
    insertAndSetFirstWall(&head, 15,  OVERALL_WINDOW_WIDTH/2-100, OVERALL_WINDOW_HEIGHT/2+50, 10, 125);//vertical gap right to robot
    insertAndSetFirstWall(&head, 16,  OVERALL_WINDOW_WIDTH/2-200, OVERALL_WINDOW_HEIGHT/2-100, 100, 10);//horizontaltop left
    insertAndSetFirstWall(&head, 17,  OVERALL_WINDOW_WIDTH/2, OVERALL_WINDOW_HEIGHT/2+200, 100, 10);
    insertAndSetFirstWall(&head, 18,  OVERALL_WINDOW_WIDTH/2-50, OVERALL_WINDOW_HEIGHT/2-150, 10, 100);
    insertAndSetFirstWall(&head, 19,  OVERALL_WINDOW_WIDTH/2, OVERALL_WINDOW_HEIGHT/2-150, 10, 100);
    insertAndSetFirstWall(&head, 20,  OVERALL_WINDOW_WIDTH/2, OVERALL_WINDOW_HEIGHT/2-150, 100, 10);
    insertAndSetFirstWall(&head, 21,  OVERALL_WINDOW_WIDTH/2+310, OVERALL_WINDOW_HEIGHT/2-200, 10, 300);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+275, OVERALL_WINDOW_HEIGHT/2-100, 10, 50);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+240, OVERALL_WINDOW_HEIGHT/2-100, 10, 50);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+240, OVERALL_WINDOW_HEIGHT/2-100, 40, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+240, OVERALL_WINDOW_HEIGHT/2-60, 40, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+175, OVERALL_WINDOW_HEIGHT/2+100, 10, 40);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+125, OVERALL_WINDOW_HEIGHT/2+100, 10, 40);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+110, OVERALL_WINDOW_HEIGHT/2+170, 10, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+200, OVERALL_WINDOW_HEIGHT/2+170, 10, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+120, OVERALL_WINDOW_HEIGHT/2+180, 80, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+150, OVERALL_WINDOW_HEIGHT/2, 10, 40);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+120, OVERALL_WINDOW_HEIGHT/2-50, 10, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+175, OVERALL_WINDOW_HEIGHT/2-50, 10, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+150, OVERALL_WINDOW_HEIGHT/2-100, 10, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+140, OVERALL_WINDOW_HEIGHT/2, 30, 10);
    insertAndSetFirstWall(&head, 22,  OVERALL_WINDOW_WIDTH/2+140, OVERALL_WINDOW_HEIGHT/2+40, 30, 10);






    setup_robot(&robot);
    updateAllWalls(head, renderer);

    SDL_Event event;
    while(!done){
        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
        SDL_RenderClear(renderer);

        //Move robot based on user input commands/auto commands
        if (robot.auto_mode == 1)
            robotAutoMotorMove(&robot, front_left_sensor, front_right_sensor);
        robotMotorMove(&robot);

        //Check if robot reaches endpoint. and check sensor values
        if (checkRobotReachedEnd(&robot, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT/2+100, 10, 100)){
            end_time = clock();
            msec = (end_time-start_time) * 1000 / CLOCKS_PER_SEC;
            robotSuccess(&robot, msec);
        }
        else if(checkRobotHitWalls(&robot, head))
            robotCrash(&robot);
        //Otherwise compute sensor information
        else {
            front_left_sensor = checkRobotSensorFrontLeftAllWalls(&robot, head);
            if (front_left_sensor>0)
                printf("Getting close on the left. Score = %d\n", front_left_sensor);

            front_right_sensor = checkRobotSensorFrontRightAllWalls(&robot, head);
            if (front_right_sensor>0)
                printf("Getting close on the right. Score = %d\n", front_right_sensor);
        }
        robotUpdate(renderer, &robot);
        updateAllWalls(head, renderer);

        // Check for user input
        SDL_RenderPresent(renderer);
        while(SDL_PollEvent(&event)){
            if(event.type == SDL_QUIT){
                done = 1;
            }
            const Uint8 *state = SDL_GetKeyboardState(NULL);
            if(state[SDL_SCANCODE_UP] && robot.direction != DOWN){
                robot.direction = UP;
            }
            if(state[SDL_SCANCODE_DOWN] && robot.direction != UP){
                robot.direction = DOWN;
            }
            if(state[SDL_SCANCODE_LEFT] && robot.direction != RIGHT){
                robot.direction = LEFT;
            }
            if(state[SDL_SCANCODE_RIGHT] && robot.direction != LEFT){
                robot.direction = RIGHT;
            }
            if(state[SDL_SCANCODE_SPACE]){
                setup_robot(&robot);
            }
            if(state[SDL_SCANCODE_RETURN]){
                robot.auto_mode = 1;
                start_time = clock();
            }
        }

        SDL_Delay(120);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    printf("DEAD\n");
}
