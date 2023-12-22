#include <Eigen/Dense>
#include <vector>

class Hash {
    public:
        Hash(double spacing, int maxNumObjects) : spacing(spacing), tableSize(2 * maxNumObjects),
                                                  cellStart(tableSize + 1), cellEntries(maxNumObjects),
                                                  queryIds(maxNumObjects), querySize(0), maxNumObjects(maxNumObjects) {}
    void create(const std::vector<Particle> particles) {
        int numObjects = std::min(static_cast<int>(particles.size()), static_cast<int>(cellEntries.size()));

        // Determine cell sizes
        cellStart.clear();
        cellEntries.clear();
        cellStart.resize(tableSize + 1, 0);
        cellEntries.resize(maxNumObjects, 0);

        for (int i = 0; i < numObjects; i++) {
            int h = hashPos(particles[i].getPosition());
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
            int h = hashPos(particles[i].getPosition());
            cellStart[h]--;
            cellEntries[cellStart[h]] = i;
        }
    }

    void query(const Particle particle,  double maxDist) {
        Eigen::Vector3d pos = particle.getPosition();
        int x0 = intCoord(pos.x() - maxDist);
        int y0 = intCoord(pos.y() - maxDist);
        int z0 = intCoord(pos.z() - maxDist);

        int x1 = intCoord(pos.x() + maxDist);
        int y1 = intCoord(pos.y() + maxDist);
        int z1 = intCoord(pos.z() + maxDist);

        querySize = 0;
        queryIds.clear();

        for (int xi = x0; xi <= x1; xi++) {
            for (int yi = y0; yi <= y1; yi++) {
                for (int zi = z0; zi <= z1; zi++) {
                    int h = hashCoords(xi, yi, zi);
                    int start = cellStart[h];
                    int end = cellStart[h + 1];

                    for (int i = start; i < end; i++) {
                        //queryIds[querySize] = cellEntries[i];
                        queryIds.push_back(cellEntries[i]);
                        querySize++;
                    }
                }
            }
        }
    }
    
    std::vector<int> queryIds;
    private:
        double spacing;
        int tableSize;
        int maxNumObjects;
        std::vector<int> cellStart;
        std::vector<int> cellEntries;
        
        int querySize;

        int hashCoords(int xi, int yi, int zi) {
            int h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481); // fantasy function
            return std::abs(h) % tableSize;
        }

        int intCoord(double coord) {
            return static_cast<int>(std::floor(coord / spacing));
        }

        int hashPos(const Eigen::Vector3d& pos) {
            return hashCoords(intCoord(pos.x()), intCoord(pos.y()), intCoord(pos.z()));
        }
};
