#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <list>
#include <memory>
#include <chrono>

using namespace std;

struct ScheduleOption {
    string instructor;
    string timeSlot;
    string classRoom;

    bool operator==(const ScheduleOption& other) const {
        return instructor == other.instructor && timeSlot == other.timeSlot && classRoom == other.classRoom;
    }
};

const vector<string> SUBJECTS = {
    "AI", "Databases", "Algorithms", "Networks", "OS",
    "Calculus", "DataStructures", "Compilers", "Ethics", "SoftwareEng"
};

const vector<string> FACULTY = {"Dr. Ada", "Dr. Babbage", "Dr. Church"};

const vector<string> TIMES = {
    "Mon 9-10", "Mon 10-11", "Mon 1-2", "Mon 2-3",
    "Tue 9-10", "Tue 10-11", "Tue 1-2", "Tue 2-3"
};

const vector<string> ROOMS = {"Room 101", "Room 102"};

map<string, vector<string>> TEACHING_QUALIFICATIONS = {
    {"Dr. Ada", {"AI", "Algorithms", "DataStructures", "Calculus"}},
    {"Dr. Babbage", {"Databases", "Networks", "OS", "SoftwareEng"}},
    {"Dr. Church", {"AI", "OS", "Algorithms", "Compilers", "Ethics"}}
};

bool canTeach(const string& instructor, const string& subject) {
    for (size_t i = 0; i < TEACHING_QUALIFICATIONS[instructor].size(); ++i) {
        if (TEACHING_QUALIFICATIONS[instructor][i] == subject) return true;
    }
    return false;
}

typedef map<string, ScheduleOption> Assignment;
typedef map<string, list<ScheduleOption>> Domain;

class CSPSolver {
public:
    struct Stats {
        long long execTimeNs;
        int backtrackCount;
        bool success;
        Stats() : execTimeNs(0), backtrackCount(0), success(false) {}
    };

    virtual pair<Assignment, Stats> solve() = 0;

    void showSchedule(const Assignment& schedule) {
        if (schedule.empty()) {
            cout << "No timetable could be generated." << endl;
            return;
        }
        cout << "--- Final Timetable ---" << endl;
        for (Assignment::const_iterator it = schedule.begin(); it != schedule.end(); ++it) {
            cout << it->first << " -> " << it->second.instructor << ", "
                 << it->second.classRoom << ", " << it->second.timeSlot << endl;
        }
    }

protected:
    Assignment schedule;
    Stats stats;

    Domain buildDomains() {
        Domain domains;
        for (size_t i = 0; i < SUBJECTS.size(); ++i) {
            string subject = SUBJECTS[i];
            for (size_t j = 0; j < FACULTY.size(); ++j) {
                string instructor = FACULTY[j];
                if (canTeach(instructor, subject)) {
                    for (size_t k = 0; k < TIMES.size(); ++k) {
                        for (size_t l = 0; l < ROOMS.size(); ++l) {
                            domains[subject].push_back({instructor, TIMES[k], ROOMS[l]});
                        }
                    }
                }
            }
        }
        return domains;
    }

    bool isValid(const string& subject, const ScheduleOption& option, const Assignment& currentAssign) {
        for (Assignment::const_iterator it = currentAssign.begin(); it != currentAssign.end(); ++it) {
            string assignedSub = it->first;
            ScheduleOption assignedOpt = it->second;

            if (assignedSub == subject) continue;
            if (assignedOpt.instructor == option.instructor && assignedOpt.timeSlot == option.timeSlot) return false;
            if (assignedOpt.classRoom == option.classRoom && assignedOpt.timeSlot == option.timeSlot) return false;
        }
        return true;
    }
};

// --- MRV Backtracking Solver ---
class MRVBacktrackingSolver : public CSPSolver {
public:
    pair<Assignment, Stats> solve() {
        auto start = chrono::high_resolution_clock::now();
        Domain domains = buildDomains();
        stats = Stats();

        stats.success = recursiveSolve(domains);

        auto end = chrono::high_resolution_clock::now();
        stats.execTimeNs = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
        return make_pair(stats.success ? schedule : Assignment(), stats);
    }

private:
    bool recursiveSolve(Domain& domains) {
        if (schedule.size() == SUBJECTS.size()) return true;

        string nextSubject = selectUsingMRV(domains);

        list<ScheduleOption>& domainList = domains[nextSubject];
        for (list<ScheduleOption>::iterator it = domainList.begin(); it != domainList.end(); ++it) {
            ScheduleOption option = *it;
            if (isValid(nextSubject, option, schedule)) {
                schedule[nextSubject] = option;
                if (recursiveSolve(domains)) return true;
                stats.backtrackCount++;
                schedule.erase(nextSubject);
            }
        }
        return false;
    }

    string selectUsingMRV(const Domain& domains) {
        string bestSubject = "";
        int smallestDomain = 1000000;
        for (size_t i = 0; i < SUBJECTS.size(); ++i) {
            string subject = SUBJECTS[i];
            if (schedule.find(subject) == schedule.end()) {
                int domainSize = (int)domains.at(subject).size();
                if (domainSize < smallestDomain) {
                    smallestDomain = domainSize;
                    bestSubject = subject;
                }
            }
        }
        return bestSubject;
    }
};

// --- Forward Checking Solver ---
class ForwardCheckingSolver : public CSPSolver {
public:
    pair<Assignment, Stats> solve() {
        auto start = chrono::high_resolution_clock::now();
        Domain domains = buildDomains();
        stats = Stats();

        stats.success = recursiveSolve(domains);

        auto end = chrono::high_resolution_clock::now();
        stats.execTimeNs = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
        return make_pair(stats.success ? schedule : Assignment(), stats);
    }

private:
    bool recursiveSolve(Domain domains) {
        if (schedule.size() == SUBJECTS.size()) return true;
        string nextSubject = pickNextSubject(domains);

        list<ScheduleOption>& options = domains[nextSubject];
        for (list<ScheduleOption>::iterator it = options.begin(); it != options.end(); ++it) {
            ScheduleOption option = *it;
            if (isValid(nextSubject, option, schedule)) {
                schedule[nextSubject] = option;
                Domain updatedDomains = domains;
                if (applyForwardCheck(nextSubject, option, updatedDomains)) {
                    if (recursiveSolve(updatedDomains)) return true;
                }
                stats.backtrackCount++;
                schedule.erase(nextSubject);
            }
        }
        return false;
    }

    string pickNextSubject(const Domain& domains) {
        for (size_t i = 0; i < SUBJECTS.size(); ++i) {
            string s = SUBJECTS[i];
            if (schedule.find(s) == schedule.end()) return s;
        }
        return "";
    }

    bool applyForwardCheck(const string& subject, const ScheduleOption& option, Domain& domains) {
        for (size_t i = 0; i < SUBJECTS.size(); ++i) {
            string otherSub = SUBJECTS[i];
            if (schedule.find(otherSub) == schedule.end()) {
                list<ScheduleOption>& opts = domains[otherSub];
                for (list<ScheduleOption>::iterator it = opts.begin(); it != opts.end();) {
                    bool conflict = (it->instructor == option.instructor && it->timeSlot == option.timeSlot) ||
                                    (it->classRoom == option.classRoom && it->timeSlot == option.timeSlot);
                    if (conflict) it = opts.erase(it);
                    else ++it;
                }
                if (opts.empty()) return false;
            }
        }
        return true;
    }
};

int main() {
    cout << "Attempting to generate a valid course timetable..." << endl;

    MRVBacktrackingSolver mrvSolver;
    cout << "\n[Using Backtracking + MRV Heuristic]" << endl;
    pair<Assignment, CSPSolver::Stats> mrvResult = mrvSolver.solve();
    Assignment mrvSchedule = mrvResult.first;
    CSPSolver::Stats mrvStats = mrvResult.second;

    mrvSolver.showSchedule(mrvSchedule);
    cout << "Stats:" << endl;
    cout << "  Success: " << (mrvStats.success ? "Yes" : "No") << endl;
    cout << "  Backtracks: " << mrvStats.backtrackCount << endl;
    cout << "  Time Taken: " << mrvStats.execTimeNs << " ns" << endl;

    ForwardCheckingSolver fcSolver;
    cout << "\n[Using Backtracking + Forward Checking]" << endl;
    pair<Assignment, CSPSolver::Stats> fcResult = fcSolver.solve();
    Assignment fcSchedule = fcResult.first;
    CSPSolver::Stats fcStats = fcResult.second;

    fcSolver.showSchedule(fcSchedule);
    cout << "Stats:" << endl;
    cout << "  Success: " << (fcStats.success ? "Yes" : "No") << endl;
    cout << "  Backtracks: " << fcStats.backtrackCount << endl;
    cout << "  Time Taken: " << fcStats.execTimeNs << " ns" << endl;

    return 0;
}
