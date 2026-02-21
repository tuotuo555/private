using System;
using System.Collections.Generic;
using Scripts.Game;
using UnityEngine;

namespace Scripts.Traffic
{
    public class StartGoalAssignments : MonoBehaviour
    {
        [Header("Build Mode")]
        public bool groupedMode = true;

        [Serializable]
        public struct Assignment
        {
            public Transform start;
            public Transform goal;
            public string groupName;
            public int indexWithinGroup;
            public int globalIndex;
        }

        private static StartGoalAssignments _instance;

        public static StartGoalAssignments Instance
        {
            get
            {
                if (_instance != null)
                {
                    return _instance;
                }

                _instance = FindFirstObjectByType<StartGoalAssignments>();
                if (_instance != null)
                {
                    return _instance;
                }

                GameObject go = new GameObject("StartGoalAssignments");
                _instance = go.AddComponent<StartGoalAssignments>();
                return _instance;
            }
        }

        private readonly List<Assignment> _assignments = new List<Assignment>();
        private readonly List<string> _groupOrderInStartsHierarchy = new List<string>();
        private readonly Dictionary<int, int> _claimIndexByInstance = new Dictionary<int, int>();
        private bool _built;
        private int _nextClaimIndex;
        private bool _usedGroupedBuild;

        public bool UsesGroupedBuild
        {
            get
            {
                BuildIfNeeded();
                return _usedGroupedBuild;
            }
        }

        public IReadOnlyList<string> GroupOrderInStartsHierarchy
        {
            get
            {
                BuildIfNeeded();
                return _groupOrderInStartsHierarchy;
            }
        }

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }

            _instance = this;
            BuildIfNeeded();
        }

        public Assignment GetAssignmentForAgent(AIP1TrafficCar agent)
        {
            BuildIfNeeded();

            if (_assignments.Count == 0)
            {
                return default;
            }

            int instanceId = agent != null ? agent.GetInstanceID() : 0;
            if (!_claimIndexByInstance.TryGetValue(instanceId, out int idx))
            {
                idx = _nextClaimIndex++;
                _claimIndexByInstance[instanceId] = idx;
            }

            if (idx < 0 || idx >= _assignments.Count)
            {
                int clamped = Mathf.Clamp(idx, 0, _assignments.Count - 1);
                idx = clamped;
            }

            return _assignments[idx];
        }

        private void BuildIfNeeded()
        {
            if (_built)
            {
                return;
            }

            _built = true;
            _assignments.Clear();
            _groupOrderInStartsHierarchy.Clear();
            _usedGroupedBuild = false;

            Transform mapRoot = ResolveMapRoot();
            if (mapRoot == null)
            {
                return;
            }

            Transform startsRoot = FindChildRecursiveByName(mapRoot, "Starts");
            Transform targetsRoot = FindChildRecursiveByName(mapRoot, "Targets");
            if (startsRoot == null || targetsRoot == null)
            {
                return;
            }

            bool startsGrouped = HasSubGroups(startsRoot);
            bool targetsGrouped = HasSubGroups(targetsRoot);
            if (groupedMode && startsGrouped && targetsGrouped)
            {
                _usedGroupedBuild = true;
                BuildGrouped(startsRoot, targetsRoot);
            }
            else
            {
                _usedGroupedBuild = false;
                BuildNonGrouped(startsRoot, targetsRoot);
            }

        }

        private void BuildNonGrouped(Transform startsRoot, Transform targetsRoot)
        {
            _groupOrderInStartsHierarchy.Add("Ungrouped");
            int n = Mathf.Min(startsRoot.childCount, targetsRoot.childCount);
            for (int i = 0; i < n; i++)
            {
                Transform s = startsRoot.GetChild(i);
                Transform g = targetsRoot.GetChild(i);
                _assignments.Add(new Assignment
                {
                    start = s,
                    goal = g,
                    groupName = "Ungrouped",
                    indexWithinGroup = i,
                    globalIndex = _assignments.Count
                });
            }

        }

        private void BuildGrouped(Transform startsRoot, Transform targetsRoot)
        {
            for (int g = 0; g < startsRoot.childCount; g++)
            {
                Transform startGroup = startsRoot.GetChild(g);
                if (startGroup != null && !_groupOrderInStartsHierarchy.Contains(startGroup.name))
                {
                    _groupOrderInStartsHierarchy.Add(startGroup.name);
                }

                Transform targetGroup = targetsRoot.Find(startGroup.name);
                if (targetGroup == null)
                {
                    continue;
                }

                int n = Mathf.Min(startGroup.childCount, targetGroup.childCount);
                for (int i = 0; i < n; i++)
                {
                    Transform s = startGroup.GetChild(i);
                    Transform t = targetGroup.GetChild(i);
                    _assignments.Add(new Assignment
                    {
                        start = s,
                        goal = t,
                        groupName = startGroup.name,
                        indexWithinGroup = i,
                        globalIndex = _assignments.Count
                    });
                }

            }
        }

        private static bool HasSubGroups(Transform root)
        {
            if (root == null || root.childCount == 0)
            {
                return false;
            }

            for (int i = 0; i < root.childCount; i++)
            {
                if (root.GetChild(i).childCount > 0)
                {
                    return true;
                }
            }

            return false;
        }

        private static Transform ResolveMapRoot()
        {
            GameObject mapManagerObject = GameObject.Find("MapManager");
            if (mapManagerObject != null)
            {
                return mapManagerObject.transform;
            }

            AbstractGameManager gm = FindFirstObjectByType<AbstractGameManager>();
            if (gm != null && gm.mapManager != null)
            {
                return gm.mapManager.transform;
            }

            return null;
        }

        private static Transform FindChildRecursiveByName(Transform root, string expectedName)
        {
            if (root == null)
            {
                return null;
            }

            if (string.Equals(root.name, expectedName, StringComparison.Ordinal))
            {
                return root;
            }

            for (int i = 0; i < root.childCount; i++)
            {
                Transform found = FindChildRecursiveByName(root.GetChild(i), expectedName);
                if (found != null)
                {
                    return found;
                }
            }

            return null;
        }
    }
}
