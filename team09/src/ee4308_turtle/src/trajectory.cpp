#include "trajectory.hpp"
#include "common.hpp"

std::vector<Position> post_process(std::vector<Position> path, Grid &grid) // returns the turning points
{
    // (1) obtain turning points
    if (path.size() <= 2)
    { // path contains 0 to elements. Nothing to process
        return path;
    }

    // add path[0] (goal) to turning_points
    std::vector<Position> turning_points = {path.front()}; 
    // add intermediate turning points
    for (int n = 2; n < path.size(); ++n)
    {
        Position &pos_next = path[n];
        Position &pos_cur = path[n - 1];
        Position &pos_prev = path[n - 2];

        double Dx_next = pos_next.x - pos_cur.x;
        double Dy_next = pos_next.y - pos_cur.y;
        double Dx_prev = pos_cur.x - pos_prev.x;
        double Dy_prev = pos_cur.y - pos_prev.y;

        // use 2D cross product to check if there is a turn around pos_cur
        if (abs(Dx_next * Dy_prev - Dy_next * Dx_prev) > 1e-5)
        {   // cross product is small enough ==> straight
            turning_points.push_back(pos_cur);
        }
    }
    // add path[path.size()-1] (start) to turning_points
    turning_points.push_back(path.back());

    std::vector<Position> post_process_path;
    
    // (2) make it more any-angle
    // done by students
    
    post_process_path = turning_points; // remove this line if (2) is done
    return post_process_path;
}

std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, double average_speed, double target_dt, Grid & grid)
{
    // (1) estimate total duration
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double duration = sqrt(Dx*Dx + Dy*Dy) / average_speed;

    // (2) generate cubic / quintic trajectory
    // done by students
    std::vector<Position> trajectory = {pos_begin};
    
    // start of cubic trajectory (uncomment this whole part and comment quintic trajectory part below, if use cubic)
    // double a0=0,a1=0,a2=0,a3=0;
    // double b0=0,b1=0,b2=0,b3=0;
    // double passing_speed=0.1; //speed at start and end
    // double v_x=0,v_y=0;

    // v_x=Dx*passing_speed / dist_euc(pos_begin,pos_end);
    // v_y=Dy*passing_speed / dist_euc(pos_begin,pos_end);   //similar triangle 

    // a0=pos_begin.x;
    // b0=pos_begin.y;
    // a1=v_x;
    // b1=v_y;
    // a2= (-3/duration/duration * pos_begin.x) - (2/duration * v_x) + (3/duration/duration * pos_end.x) - (1/duration * v_x);
    // b2= (-3/duration/duration * pos_begin.y) - (2/duration * v_y) + (3/duration/duration * pos_end.y) - (1/duration * v_y);
    // a3= (2/duration/duration/duration * pos_begin.x) + (1/duration/duration * v_x) - (2/duration/duration/duration * pos_end.x) + (1/duration/duration * v_x);
    // b3= (2/duration/duration/duration * pos_begin.y) + (1/duration/duration * v_y) - (2/duration/duration/duration * pos_end.y) + (1/duration/duration * v_y);
    
    // for (double time = target_dt; time < duration; time += target_dt)
    // {
    //     trajectory.emplace_back(
    //         a0 + a1*time + a2*time*time + a3*time*time*time,
    //         b0 + b1*time + b2*time*time + b3*time*time*time
    //     );
    // } // end of cubic trajectory

    // Start of quintic trajectory
    double a0=0,a1=0,a2=0,a3=0,a4=0,a5=0;
    double b0=0,b1=0,b2=0,b3=0,b4=0,b5=0;
    double passing_speed=0.1,passing_acc=0; //speed and acceleration at start and end
    double v_x=0,v_y=0;
    double acc_x=0,acc_y=0;
    double length=dist_euc(pos_begin,pos_end);

    v_x=Dx*passing_speed / length;
    v_y=Dy*passing_speed / length;   //similar triangle 
    acc_x=Dx*passing_acc / length;
    acc_y=Dy*passing_acc / length;   //acc = 0 as passing_acc set to 0 for simplicity

    a0=pos_begin.x;
    b0=pos_begin.y;
    a1=v_x;
    b1=v_y;
    a2= 0.5 * acc_x;
    b2= 0.5 * acc_y;
    a3= (-10/pow(duration,3) * pos_begin.x) - (6/pow(duration,2) * v_x) - (3/2/duration * acc_x) + (10/pow(duration,3) * pos_end.x) - (4/pow(duration,2) * v_x) + (1/2/duration * acc_x);
    b3= (-10/pow(duration,3) * pos_begin.y) - (6/pow(duration,2) * v_y) - (3/2/duration * acc_y) + (10/pow(duration,3) * pos_end.y) - (4/pow(duration,2) * v_y) + (1/2/duration * acc_y);
    a4= (15/pow(duration,4) * pos_begin.x) + (8/pow(duration,3) * v_x) + (3/2/pow(duration,2) * acc_x) + (-15/pow(duration,4) * pos_end.x) + (7/pow(duration,3) * v_x) + (-1/pow(duration,2) * acc_x);
    b4= (15/pow(duration,4) * pos_begin.y) + (8/pow(duration,3) * v_y) + (3/2/pow(duration,2) * acc_y) + (-15/pow(duration,4) * pos_end.y) + (7/pow(duration,3) * v_y) + (-1/pow(duration,2) * acc_y);
    a5= (-6/pow(duration,5) * pos_begin.x) + (-3/pow(duration,4) * v_x) + (-1/2/pow(duration,3) * acc_x) + (6/pow(duration,5) * pos_end.x) + (-3/pow(duration,4) * v_x) + (1/2/pow(duration,3) * acc_x);
    b5= (-6/pow(duration,5) * pos_begin.y) + (-3/pow(duration,4) * v_y) + (-1/2/pow(duration,3) * acc_y) + (6/pow(duration,5) * pos_end.y) + (-3/pow(duration,4) * v_y) + (1/2/pow(duration,3) * acc_y);

    for (double time = target_dt; time < duration; time += target_dt)
    {
        trajectory.emplace_back(
            a0 + a1*time + a2*pow(time,2) + a3*pow(time,3)+a4*pow(time,4)+a5*pow(time,5),
            b0 + b1*time + b2*pow(time,2) + b3*pow(time,3)+b4*pow(time,4)+b5*pow(time,5)
        );
    } // end of quintic trajectory

    // OR (2) generate targets for each target_dt
    // std::vector<Position> trajectory = {pos_begin};
    // for (double time = target_dt; time < duration; time += target_dt)
    // {
    //     trajectory.emplace_back(
    //         pos_begin.x + Dx*time / duration,
    //         pos_begin.y + Dy*time / duration
    //     );
    // }

    return trajectory; 
}

bool is_safe_trajectory(std::vector<Position> trajectory, Grid & grid)
{   // returns true if the entire path is accessible; false otherwise
    if (trajectory.size() == 0)
    {   // no path
        return false; 
    } 
    else if (trajectory.size() == 1)
    {   // goal == start
        return grid.get_cell(trajectory.front()); // depends on the only cell in the path
    }

    // if there are more than one turning points. Trajectory must be fine enough.
    for (int n=1; n<trajectory.size(); ++n)
    {
        if (!grid.get_cell(trajectory[n]))
            return false;
        /* // Use this if the trajectory points are not fine enough (distance > cell_size)
        Index idx_src = grid.pos2idx(trajectory[n-1]);
        Index idx_tgt = grid.pos2idx(trajectory[n]);
        
        grid.los.reset(idx_src, idx_tgt); // interpolate a straight line between points; can do away with los if points are fine enough.
        Index idx = idx_src;
        while (idx.i != idx_tgt.i || idx.j != idx_tgt.j)
        {
            if (!grid.get_cell(idx))
            {
                return false;
            }
            idx = grid.los.next();
        }
        */
    }
    return true;
}