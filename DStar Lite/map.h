/**
 * Map.
 *
 * @package		DStarLite
 * @author		Yanlong Wang <yanlong.wangchn@gmail.com>
 * @license		MIT
 */
#ifndef DSTARLITE_MAP_H
#define DSTARLITE_MAP_H

#include <functional>
#include <stdlib.h>

#include "math.h"

using namespace std;

class Map
{
	public:

		class Cell
		{
			public:

				/**
					* Hash
					*/
				class Hash : public unary_function<Cell*, size_t>
				{
					public:

						/**
							* @var  int  hash "constant" (may need to change if width exceeds this value)
							*/
						static const int C;
						static const int D;

						/**
							* Hashes cell based on coordinates.
							*
							* @param   Cell*
							* @return  size_t
							*/
						// wyl: unary_function is just a base class, from which specific unary function objects are derived.
						// wyl: Generically, function objects are instances of a class with member function operator() defined. 
						size_t operator()(Cell* c) const;
				};

				/**
					* @var  static const int  number of cell neighbors
					*/
				static const unsigned int NUM_NBRS;

				/**
					* @var  static const double  cost of an unwalkable cell
					*/
				static const double COST_UNWALKABLE;

				/**
					* @var  double  cost of cell
					*/
				double cost;

				/**
				* Gets expansion state.
				*
				* @return  unsigned int
				*/
				bool expandedStatus;


				/**
					* Constructor.
					*
					* @param   unsigned int        x-coordinate
					* @param   unsigned int        y-coordinate
					* @param   unsigned int        z-coordinate
					* @param   double [optional]   cost of the cell
					*/
				Cell(unsigned int x, unsigned int y, unsigned int z, double cost = 1.0);

				/**
					* Deconstructor.
					*/
				~Cell();

				/**
					* Initialize.
					*
					* @param   Cell**  cell neighbors
					* @return  void
					*/
				void Map::Cell::init(Cell** pres, Cell** sucs);

				/**
					* Gets cell neighbors.
					*
					* @return  Cell**
					*/
				Cell** pres();

				/**
					* Gets cell neighbors.
					*
					* @return  Cell**
					*/
				Cell** sucs();

				/**
					* Gets x-coordinate.
					*
					* @return  unsigned int
					*/
				unsigned int x();

				/**
					* Gets y-coordinate.
					*
					* @return  unsigned int
					*/
				unsigned int y();

				/**
				* Gets z-coordinate.
				*
				* @return  unsigned int
				*/
				unsigned int z();

				

			protected:

				/**
					* @var  bool  initialized
					*/
				bool _init;

				/**
					* @var  Cell**  predecessors
					*/
				Cell** _pres;

				/**
				* @var  Cell**  successors
				*/
				Cell** _sucs;

				/**
					* @var  unsigned int  x-coordinate
					*/
				unsigned int _x;

				/**
					* @var  unsigned int  y-coordinate
					*/
				unsigned int _y;

				/**
					* @var  unsigned int  y-coordinate
					*/
				unsigned int _z;
		};

		/**
			* Constructor.
			*
			* @param  unsigned int   rows
			* @param  unsigned int   columns
			* @param  unsigned int   tiers
			*/
		Map(unsigned int rows, unsigned int cols, const unsigned int tiers);

		/**
			* Deconstructor.
			*/
		~Map();

		/**
			* Retrieves a cell.
			*
			* @param   unsigned int   row
			* @param   unsigned int   column
			* @param   unsigned int   tier
			* @return  Map::Cell*
			*/
		Cell* operator()(const unsigned int row, const unsigned int col, const unsigned int tier);

		/**
			* Gets number of cols.
			*
			* @return  unsigned int
			*/
		unsigned int cols();

		/**
			* Checks if row/col exists.
			*
			* @param   unsigned int   row
			* @param   unsigned int   col
			* @param   unsigned int   tier
			* @return  bool
			*/
		bool has(unsigned int row, unsigned int col, const unsigned int tier);

		/**
			* Gets number of rows.
			*
			* @return  unsigned int
			*/
		unsigned int rows();

		/**
		* Gets number of tiers.
		*
		* @return  unsigned int
		*/
		unsigned int tiers();

protected:
			
		/**
			* @var  Cell***  cells of the map
			*/
		Cell**** _cells;

		/**
			* @var  unsigned int columns
			*/
		unsigned int _cols;

		/**
			* @var  unsigned int  rows
			*/
		unsigned int _rows;

		/**
			* @var  unsigned int  rows
			*/
		unsigned int _tiers;
};



#endif // DSTARLITE_MAP_H
