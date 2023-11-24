#include <Eigen/Dense>
#include <vector>

class Hash {
    Hash(double spacing, int maxNumObjects) : spacing(spacing), tableSize(2 * maxNumObjects),
                                              cellStart(tableSize + 1), cellEntries(maxNumObjects),
                                              queryIds(maxNumObjects), querySize(0) {}

    void create(const Eigen::MatrixXd& pos) {
        int numObjects = std::min(static_cast<int>(pos.rows()), static_cast<int>(cellEntries.size()));

        // Determine cell sizes
        cellStart.clear();
        cellEntries.clear();

        for (int i = 0; i < numObjects; i++) {
            int h = hashPos(pos, i);
            cellStart[h]++;
        }

        // Determine cell starts
        int start = 0;
        for (int i = 0; i < tableSize; i++) {
            start += cellStart[i];
            cellStart[i] = start;
        }
        cellStart[tableSize] = start; // guard

        // Fill in objects ids
        for (int i = 0; i < numObjects; i++) {
            int h = hashPos(pos, i);
            cellStart[h]--;
            cellEntries[cellStart[h]] = i;
        }
    }

    void query(const Eigen::MatrixXd& pos, int nr, double maxDist) {
        int x0 = intCoord(pos(nr, 0) - maxDist);
        int y0 = intCoord(pos(nr, 1) - maxDist);
        int z0 = intCoord(pos(nr, 2) - maxDist);

        int x1 = intCoord(pos(nr, 0) + maxDist);
        int y1 = intCoord(pos(nr, 1) + maxDist);
        int z1 = intCoord(pos(nr, 2) + maxDist);

        querySize = 0;

        for (int xi = x0; xi <= x1; xi++) {
            for (int yi = y0; yi <= y1; yi++) {
                for (int zi = z0; zi <= z1; zi++) {
                    int h = hashCoords(xi, yi, zi);
                    int start = cellStart[h];
                    int end = cellStart[h + 1];

                    for (int i = start; i < end; i++) {
                        queryIds[querySize] = cellEntries[i];
                        querySize++;
                    }
                }
            }
        }
    }
    private:
        double spacing;
        int tableSize;
        std::vector<int> cellStart;
        std::vector<int> cellEntries;
        std::vector<int> queryIds;
        int querySize;

        int hashCoords(int xi, int yi, int zi) {
            int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481); // fantasy function
            return std::abs(h) % tableSize;
        }

        int intCoord(double coord) {
            return static_cast<int>(std::floor(coord / spacing));
        }

        int hashPos(const Eigen::MatrixXd& pos, int nr) {
            return hashCoords(intCoord(pos(nr, 0)), intCoord(pos(nr, 1)), intCoord(pos(nr, 2)));
        }
};
