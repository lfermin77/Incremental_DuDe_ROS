#pragma once
#ifndef _KNAPSACK_H
#define _KNAPSACK_H

#include "cutset.h"
#include "diagonal2.h"
#include "polygon.h"

class c_knapsack
{
public:

    c_knapsack(vector<c_cutset>& all_cutsets);

	//build knapsack table
	void build();       //max size without given a number
    void build(uint n); //with size n

    //storoe cuts in "cuts" and return the score
    double getOptimalCuts(list<ply_vertex*>& pms, double tau, vector<c_diagonal>& cuts);

protected:

    struct cell{
        cell(){ score=0; }
        vector<c_cutset> cutsets;
        void computeScore();            //compute the total score of cutsets
        void addCutset(c_cutset& cset); //add cset to cusets
        void unionCuts( vector<c_diagonal>& diagonals ); //compute the unions of all cuts in cutsets
        double score;
    };

    void initTable();//initialize the table

    void updateCell(uint n, uint m, uint iteration);//update the solution in cell n,m

    //check if the given cset can be add to c without exceeding capacity
    bool enoughCapacity(uint capacity, const cell& c, c_cutset& cset);

    // update c by add cset to it
    //
    // reject the cutsets in c that conflict with cset
    // update the score of the cutsets as the total scores to the cuts
    //
    void add2Cell(cell& c, c_cutset& cset);

private:

    vector<c_cutset> m_all_cutsets;
    vector< vector<cell> > m_table;

    uint m_N;//capacity desired number of cuts
    uint m_M;//number of items (number of cut sets)
};

/*

typedef vector< pair<int,int> > optimal_solution;

class knapsack 
{
public:
	vector<cut_set> cut_sets;//size is M (all items)

	//define a 2D array M*N

	vector< vector<double> > scores;//save the total score

	vector< vector<optimal_solution> > optimal_solutions;

	int N;//capacity desired number of cuts
	int M;//number of items (number of cut sets)

	void getItems(vector<pocket_minimum>& pocket_minima);
	void initTable();//initialize the table of scores
	double getIncresed_score(vector< pair<int,int> > cuts_m, cut_set cut_set_m, int k, int m, int n);
	void fillTable();//fill out every table item
};
*/

#endif
