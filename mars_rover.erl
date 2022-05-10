-module(mars_rover).

-export([start/2, init/2, rover_controller/3, controller_add_robot/6, location/1, update_location/5, rover/2, rover/5]).

-record(state, {
    robots
}).

-record(rover_state, {
    robot
}).

controller_node() -> 
    controller@TamsanqasMacBookPro.


%%% CONTROLLER PROTOCOLS
start(X, Y) ->
    io:format("-----------Starting the Mars Rover Controller------------- ~n"),
    Pid = spawn(?MODULE, init, [X, Y]),
    register(controller, Pid),
    controller ! {X, Y}.
    
init(X, Y) ->
    process_flag(trap_exit, true),
    Robots = dict:new(),
    State = #state{robots = Robots},
    rover_controller(State, X, Y).

rover_controller(#state{
        robots = Robots  
    } = State, Xaxis, Yaxis) ->
    receive
        {Xcoord, Ycoord} ->
            io:format("This is the grid size of the map: ~n"),
            io:format("X: ~p by Y: ~p ~n", [Xcoord, Ycoord]),
            rover_controller(State, Xaxis, Yaxis);
        {location, From, Name, X, Y, Direction} ->
            New_Robot_List = controller_add_robot(From, Name, X, Y, Direction, Robots),
            From ! {controller, robot_added},
            rover_controller(New_Robot_List, Xaxis, Yaxis);
        {update_location, Name, Xnew, Ynew, Direction, Directions_Sequence} ->
            % find the rover to update and send the instructions to move it
            CurrentRoverPid = get_current_rover_pid(Name, Robots),
            CurrentRoverPid ! {Name, Xnew, Ynew, Direction, Directions_Sequence, Xaxis, Yaxis},
            rover_controller(State, Xaxis, Yaxis);
        {lost, Name, X, Y, Direction} ->
            io:format("Rover ~p last position (~p, ~p) moving ~p is now lost", [Name, X, Y, Direction]);
        {'EXIT', From, Reason} ->
            error_logger:error_msg("Rover lost, got: ~p~n", [{'EXIT', From, Reason}])
    after 5000 -> 
         {error, timeout}
    end.

get_current_rover_pid(Name, Robots) ->
    case dict:find(Name, Robots) of
        error -> 
            {error, "There is no such rover on mars planet."};
        % {ok, {From, X, Y, Direction}} -> 
        {ok, {From, _, _, _}} -> 
            From
    end.

controller_add_robot(From, Name, X, Y, Direction, Robots) ->
    dict:store(
        Name, 
        {From, X, Y, Direction}, 
        Robots
    ).

get_current_x_coord_from_controller(Name, Robots) ->
    case dict:find(Name, Robots) of 
        error -> 0;
        % {ok, {From, X0, Y0, Direction0}} -> X0
        {ok, {_, X0, _, _}} -> X0
    end.

get_current_y_coord_from_controller(Name, Robots) ->
    case dict:find(Name, Robots) of
        error -> 0;
        % {ok, {From, X0, Y0, Direction0}} -> Y0
        {ok, {_, _, Y0, _}} -> Y0
    end.


%%% ROVER PROTOCOLS
location(Name) ->
    spawn(?MODULE, rover, [controller_node(), Name, 0, 0, "N"]).

update_location(Name, Xnew, Ynew, Direction, [] = Directions_Sequence) ->
     controller ! {update_location, Name, Xnew, Ynew, Direction, Directions_Sequence}.

rover(Controller_Node, Name, X, Y, Direction) ->
    {controller, Controller_Node} ! {location, self(), Name, X, Y, Direction},
    await_result(),
    Robot = dict:new(),
    Rover_State = #rover_state{robot = Robot},
    rover(Controller_Node, Rover_State).

rover(Controller_Node, #rover_state{
        robot = Robot  
    } = Rover_State) ->
    receive
        {Name, Xnew, Ynew, Direction, Directions_Sequence, Xaxis, Yaxis} ->
            move_rover(Name, Xnew, Ynew, Direction, Rover_State, Directions_Sequence, Xaxis, Yaxis),
            rover(Controller_Node, Rover_State)
    after 5000 -> 
         {error, timeout}
    end.

move_rover(Name, Xnew, Ynew, Direction, Rover_State, Directions_Sequence, Xaxis, Yaxis) ->

    Number_Of_Direction_Sequences = string:length(Directions_Sequence),
    loop(Number_Of_Direction_Sequences, Name, Xnew, Ynew, Direction, Rover_State, Directions_Sequence, Xaxis, Yaxis).
    loop(0, Name, Xfinal, Yfinal, Direction_final, Rover_State, _, Xaxis, Yaxis) ->
        {controller, controller_node()} ! {current_location, Xfinal, Yfinal, Direction_final};
    loop(Count, Name, X, Y, Direction, #rover_state{robot=Robot}=Rover_State, Directions_Sequence, Xaxis, Yaxis) ->
        % Move the rover in a specific direction and update the server
        case string:uppercase(read_direction_sequence_value(Directions_Sequence, Count)) of 
            "L" ->
                io:format("Turning left 90 degrees"),
                case string:uppercase(Direction) of 
                    "N" ->
                        Robot1 = dict:store(
                            Name, 
                            {X, Y, "W"},
                            Robot
                        )
                end, 
                Rover_State#rover_state{robot = Robot1},
                
                case string:uppercase(Direction) of 
                    "E" -> 
                        Robot2 = dict:store(
                            Name, 
                            {X, Y, "S"},
                            Robot
                        )
                end,
                Rover_State#rover_state{robot = Robot2},

                case string:uppercase(Direction) of 
                    "S" ->
                        Robot3 = dict:store(
                            Name, 
                            {X, Y, "W"},
                            Robot
                        ) 
                end,
                Rover_State#rover_state{robot = Robot3},

                case string:uppercase(Direction) of 
                    "W" ->
                        Robot4 = dict:store(
                            Name, 
                            {X, Y, "N"},
                            Robot
                        ) 
                end,
                Rover_State#rover_state{robot = Robot4};

            "R" ->
                io:format("Turning Right 90 degrees"),
                case string:uppercase(Direction) of 
                    "N" -> 
                        Robot5 = dict:store(
                            Name, 
                            {X, Y, "W"},
                            Robot
                        )  
                end, 
                Rover_State#rover_state{robot = Robot5},

                case string:uppercase(Direction) of 
                    "W" -> 
                        Robot6 = dict:store(
                            Name, 
                            {X, Y, "S"},
                            Robot
                        )  
                end,
                Rover_State#rover_state{robot = Robot6},

                case string:uppercase(Direction) of 
                    "S" ->
                        Robot7 = dict:store(
                            Name, 
                            {X, Y, "E"},
                            Robot
                        )
                end,
                Rover_State#rover_state{robot = Robot7},

                case string:uppercase(Direction) of 
                    "E" ->
                        Robot8 = dict:store(
                            Name, 
                            {X, Y, "N"},
                            Robot
                        )
                end,
                Rover_State#rover_state{robot = Robot8};

            "F" ->
                io:format("Moving Forwards One Step"),
                case string:uppercase(Direction) of 
                    "N" ->
                        % check if the the coordinate is going to go over the grid on the Y Axis
                        case (Y + 1) =< Yaxis of
                            true ->
                                Robot9 = dict:store(
                                    Name, 
                                    {X, Y + 1, Direction},
                                    Robot
                                ),
                                Rover_State#rover_state{robot = Robot9};
                            false ->
                                {controller, controller_node()} ! {lost, Name, X, Y, Direction},
                                exit(normal)
                        end   
                end,
                
                case string:uppercase(Direction) of 
                    "E" ->
                        % check if the the coordinate is going to go over the grid on the X Axis
                        case (X + 1) =< Xaxis of
                            true ->
                                Robot10 = dict:store(
                                    Name, 
                                    {X+1, Y, Direction},
                                    Robot
                                ),
                                Rover_State#rover_state{robot = Robot10};
                            false ->
                                {controller, controller_node()} ! {lost, Name, X, Y, Direction},
                                exit(normal)
                        end
                end,
                case string:uppercase(Direction) of 
                    "S" ->
                        % check if the the coordinate is going to go over the grid on the Y Axis
                        case (Y - 1) >= 0 of
                            true ->
                                Robot11 = dict:store(
                                    Name, 
                                    {X, Y - 1, Direction},
                                    Robot
                                ),
                                Rover_State#rover_state{robot = Robot11};
                            false ->
                                {controller, controller_node()} ! {lost, Name, X, Y, Direction},
                                exit(normal)
                        end
                end,
                case string:uppercase(Direction) of 
                    "W" ->
                        % check if the the coordinate is going to go over the grid on the Y Axis
                        case (X - 1) >= 0 of
                            true ->
                                Robot12 = dict:store(
                                    Name, 
                                    {X - 1, Y, Direction},
                                    Robot
                                ),
                                Rover_State#rover_state{robot = Robot12};
                            false ->
                                {controller, controller_node()} ! {lost, Name, X, Y, Direction},
                                exit(normal)
                        end
                end;
            "R" ->
                io:format("Moving Backwards One Step"),
                case string:uppercase(Direction) of 
                    "N" ->
                        % check if the the coordinate is going to go over the grid on the Y Axis
                        case (Y - 1) >= 0 of
                            true ->
                                Robot13 = dict:store(
                                    Name, 
                                    {X, Y - 1, Direction},
                                    Robot
                                ),
                                Rover_State#rover_state{robot = Robot13};
                            false ->
                                {controller, controller_node()} ! {lost, Name, X, Y, Direction},
                                exit(normal)
                        end
                end,
                case string:uppercase(Direction) of 
                    "E" ->
                        % check if the the coordinate is going to go over the grid on the X Axis
                        case (X - 1) >= 0 of
                            true ->
                                Robot14 = dict:store(
                                    Name, 
                                    {X-1, Y, Direction},
                                    Robot
                                ),
                                Rover_State#rover_state{robot = Robot14};
                            false ->
                                {controller, controller_node()} ! {lost, Name, X, Y, Direction},
                                exit(normal)
                        end
                end,
                case string:uppercase(Direction) of 
                    "S" ->
                        % check if the the coordinate is going to go over the grid on the X Axis
                        case (Y + 1) =< Yaxis of
                            true ->
                                Robot15 = dict:store(
                                    Name, 
                                    {X, Y + 1, Direction},
                                    Robot
                                ),
                                Rover_State#rover_state{robot = Robot15};
                            false ->
                                {controller, controller_node()} ! {lost, Name, X, Y, Direction},
                                exit(normal)
                        end
                end,
                case string:uppercase(Direction) of 
                    "W" ->
                        % check if the the coordinate is going to go over the grid on the X Axis
                        case (X + 1) =< Xaxis of
                            true ->
                                Robot16 = dict:store(
                                    Name, 
                                    {X + 1, Y, Direction},
                                    Robot
                                ),
                                Rover_State#rover_state{robot = Robot16};
                            false ->
                                {controller, controller_node()} ! {lost, Name, X, Y, Direction},
                                exit(normal)
                        end
                end
            
        end,
        loop(Count-1, Name, X, Y, Direction, Rover_State, Directions_Sequence, Xaxis, Yaxis).

read_direction_sequence_value(Directions_Sequence, Count) ->
    Reversed_Directions_Sequence = lists:reverse(Directions_Sequence),
    string:slice(Reversed_Directions_Sequence, Count, 1).

%%% Find X and Y coord from the rover itself
get_current_x_coord_from_rover(Name, Robot) ->
    case dict:find(Name, Robot) of 
        error -> 0;
        % {ok, {From, X0, Y0, Direction0}} -> X0
        {ok, {_, X0, _, _}} -> X0
    end.

get_current_y_coord_from_rover(Name, Robot) ->
    case dict:find(Name, Robot) of
        error -> 0;
        % {ok, {From, X0, Y0, Direction0}} -> Y0
        {ok, {_, _, Y0, _}} -> Y0
    end.

get_current_direction_from_rover(Name, Robot) ->
    case dict:find(Name, Robot) of 
        error -> 0;
        {ok, {_, _, _, Direction}} -> Direction
    end.

await_result() ->
    receive
        {controller, stop, Why} ->
            io:format("~p~n", [Why]),
            exit(normal);
        {controller, What} ->
            io:format("~p~n", [What])
    after 5000 -> 
         {error, timeout}
    end.

