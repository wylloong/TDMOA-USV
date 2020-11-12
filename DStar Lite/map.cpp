/**
 * Map.
 *
 * @package		DStarLite
 * @author		Aaron Zampaglione <azampagl@gmail.com>
 * @copyright	Copyright (C) 2011 Aaron Zampaglione
 * @license		MIT
 */
#include "map.h"

using namespace std;

/**
 * @var  unsigned int  number of cell neighbors
 */
const unsigned int Map::Cell::NUM_NBRS = 5;

/**
 * @var  double  cost of an unwalkable tile
 */
const double Map::Cell::COST_UNWALKABLE = DBL_MAX;

/**
 * @var  int  hash "constant" (may need to change if map width exceeds this value)
 */
const int Map::Cell::Hash::C = 1000000;
const int Map::Cell::Hash::D = 1000;


/**
 * Constructor.
 *
 * @param  unsigned int   rows
 * @param  unsigned int   columns
 */
Map::Map(unsigned int rows, unsigned int cols, unsigned int tiers)
{
	_rows = rows;
	_cols = cols;

	_cells = new Cell***[rows]; // wyl:构建二维数组，保持cell的x和y坐标

	for (unsigned int i = 0; i < rows; i++)
	{
		_cells[i] = new Cell**[cols];

		for (unsigned int j = 0; j < cols; j++)
		{
			_cells[i][j] = new Cell*[tiers];
			for (unsigned int k = 0; k < tiers; k++)
			{
				// Initialize cells
				_cells[i][j][k] = new Cell(i, j, k);
			}
		}
	}

	// Attach neighbors
	for (unsigned int i = 0; i < rows; i++)
	{
		for (unsigned int j = 0; j < cols; j++)
		{
			for (unsigned int k = 0; k < tiers; k++)
			{
				Cell** pres = new Cell*[Cell::NUM_NBRS];
				Cell** sucs = new Cell*[Cell::NUM_NBRS];
				for (unsigned int m = 0; m < Cell::NUM_NBRS; m++)
				{
					pres[m] = NULL;
					sucs[m] = NULL;
				}

				// bottom - predecessors
				if (k != 0)
				{
					// Top
					if (i != 0)
					{
						//if (j != 0)
						//{
						//	// Top left
						//	pres[0] = _cells[i - 1][j - 1][k-1];
						//}

						// Top middle
						pres[0] = _cells[i - 1][j][k - 1];

						//if (j < cols - 1)
						//{
						//	// Top right
						//	pres[2] = _cells[i - 1][j + 1][k - 1];
						//}
					}

					if (j < cols - 1)
					{
						// Middle right
						pres[1] = _cells[i][j + 1][k - 1];
					}

					// Bottom
					if (i < rows - 1)
					{
						//if (j < cols - 1)
						//{
						//	// Bottom right
						//	pres[4] = _cells[i + 1][j + 1][k - 1];
						//}

						// Bottom middle
						pres[2] = _cells[i + 1][j][k - 1];

						//if (j != 0)
						//{
						//	// Bottom left
						//	pres[6] = _cells[i + 1][j - 1][k - 1];
						//}
					}

					if (j != 0)
					{
						// Middle left
						pres[3] = _cells[i][j - 1][k - 1];
					}
					pres[4] = _cells[i][j][k - 1];
				}
				
				// Top - successors
				if (k < tiers-1)
				{
					// Top
					if (i != 0)
					{
						//if (j != 0)
						//{
						//	// Top left
						//	sucs[0] = _cells[i - 1][j - 1][k + 1];
						//}

						// Top middle
						sucs[0] = _cells[i - 1][j][k + 1];

						//if (j < cols - 1)
						//{
						//	// Top right
						//	sucs[2] = _cells[i - 1][j + 1][k + 1];
						//}
					}

					if (j < cols - 1)
					{
						// Middle right
						sucs[1] = _cells[i][j + 1][k + 1];
					}

					// Bottom
					if (i < rows - 1)
					{
						//if (j < cols - 1)
						//{
						//	// Bottom right
						//	sucs[4] = _cells[i + 1][j + 1][k + 1];
						//}

						// Bottom middle
						sucs[2] = _cells[i + 1][j][k + 1];

						//if (j != 0)
						//{
						//	// Bottom left
						//	sucs[6] = _cells[i + 1][j - 1][k + 1];
						//}
					}

					if (j != 0)
					{
						// Middle left
						sucs[3] = _cells[i][j - 1][k + 1];
					}
					sucs[4] = _cells[i][j][k + 1];
				}

				// 为每一个网格附上子节点
				_cells[i][j][k]->init(pres, sucs);
			}
		}
	}
}

/**
 * Deconstructor.
 */
Map::~Map()
{
	for (unsigned int i = 0; i < _rows; i++)
	{
		for (unsigned int j = 0; j < _cols; j++)
		{
			for (unsigned int k = 0; k < _tiers; k++)
			{
				delete _cells[i][j][k];
			}
			delete[] _cells[i][j];
		}
		delete[] _cells[i];
	}
	delete[] _cells;
}

/**
 * Retrieves a cell.
 *
 * @param   unsigned int   row
 * @param   unsigned int   column
 * @return  Map::Cell*
 */
Map::Cell* Map::operator()(const unsigned int row, const unsigned int col, const unsigned int tier)
{
	return _cells[row][col][tier];
}

/**
 * Gets number of cols.
 *
 * @return  unsigned int
 */
unsigned int Map::cols()
{
	return _cols;
}

/**
* Gets number of rows.
*
* @return  unsigned int
*/
unsigned int Map::rows()
{
	return _rows;
}

/**
* Gets number of tiers.
*
* @return  unsigned int
*/
unsigned int Map::tiers()
{
	return _tiers;
}


/**
 * Checks if row/col exists.
 *
 * @param   unsigned int   row
 * @param   unsigned int   col
 * @return  bool
 */
bool Map::has(unsigned int row, unsigned int col, unsigned int tier)
{
	return (row >= 0 && row < _rows && col >= 0 && col < _cols && tier >= 0 && tier < _tiers);
}



/**
 * Constructor.
 *
 * @param   unsigned int        x-coordinate
 * @param   unsigned int        y-coordinate
 * @param   unsigned int        z-coordinate
 * @param   double [optional]   cost of the cell
 */				
Map::Cell::Cell(unsigned int x, unsigned int y, unsigned int z, double cost)
{
	_init = false;

	_pres = NULL;
	_sucs = NULL;

	_x = x;
	_y = y;
	_z = z;
	expandedStatus = false;
	this->cost = cost;
}

/**
 * Deconstructor.
 */
Map::Cell::~Cell()
{
	if (_pres != NULL)
		delete[] _pres;
	if (_sucs != NULL)
		delete[] _sucs;
}

/**
 * Initialize.
 *
 * @param   Cell**  cell neighbors
 * @return  void
 */
void Map::Cell::init(Cell** pres, Cell** sucs)
{
	if (_init)
		return;
	// 之后直接退出
	_init = true;

	_pres = pres;
	_sucs = sucs;
}

/**
 * Gets cell neighbors.
 *
 * @return  Cell**
 */
Map::Cell** Map::Cell::pres()
{
	return _pres;
}

Map::Cell** Map::Cell::sucs()
{
	return _sucs;
}

/**
 * Gets x-coordinate.
 *
 * @return  unsigned int
 */
unsigned int Map::Cell::x()
{
	return _x;
}

/**
 * Gets y-coordinate.
 *
 * @return  unsigned int
 */
unsigned int Map::Cell::y()
{
	return _y;
}

/**
* Gets y-coordinate.
*
* @return  unsigned int
*/
unsigned int Map::Cell::z()
{
	return _z;
}

//// 扩展属性，用于后续的性能分析和统计
//bool Map::Cell::expanded()
//{
//	return _expanded;
//}

/**
 * Hashes cell based on coordinates.
 *
 * @param   Cell*
 * @return  size_t
 */
size_t Map::Cell::Hash::operator()(Cell* c) const
{
	return Cell::Hash::C * c->y() + Cell::Hash::D * c->x()+c->z();
}