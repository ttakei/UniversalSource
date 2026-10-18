using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace UniversalSource
{
    /// <summary>
    /// 2つの <see cref="SourceNode"/> を結ぶ向き付きの接続。
    /// 値は端点の世界座標ではなく、このリンクが持つ補正行列で相手の枠へ写す。
    /// 保存しているのは静止時の補正行列で、転送時は両端のスピン位相を非可換に掛ける。
    /// A→B の実効行列はスピン結合 × 静止行列、B→A はその向きで組み直す。
    /// </summary>
    [DisallowMultipleComponent]
    [AddComponentMenu("UniversalSource/Universal Link")]
    public sealed class UniversalLink : MonoBehaviour
    {
        const float SingularDeterminant = 1e-6f;

        [SerializeField] SourceNode endpointA;
        [SerializeField] SourceNode endpointB;

        // 未設定の Matrix4x4 はゼロ行列としてシリアライズされる。identity を明示する。
        [SerializeField] Matrix4x4 transportMatrix = Matrix4x4.identity;

        SourceNode attachedA;
        SourceNode attachedB;

        public SourceNode EndpointA => endpointA;
        public SourceNode EndpointB => endpointB;

        /// <summary>フィードバックが更新する静止補正。スピンは含まない。</summary>
        public Matrix4x4 RestMatrix => transportMatrix;

        /// <summary>endpointA から endpointB へ運ぶ実効補正。両端の位相で変化する。</summary>
        public Matrix4x4 TransportMatrix
        {
            get { return Effective(endpointA, endpointB, transportMatrix); }
        }

        void Reset()
        {
            transportMatrix = Matrix4x4.identity;
        }

        void Awake()
        {
            EnsureTransport();
        }

        void OnEnable()
        {
            Attach();
        }

        void OnDisable()
        {
            Detach();
        }

        /// <summary>2ノードを結び、A→B の補正行列を設定して両端の近傍へ登録する。</summary>
        public bool Bind(SourceNode a, SourceNode b, Matrix4x4 transportAtoB)
        {
            if (a == null || b == null || a == b)
                return false;

            Detach();
            endpointA = a;
            endpointB = b;
            transportMatrix = SourceNode.IsZeroMatrix(transportAtoB) ? Matrix4x4.identity : transportAtoB;
            Attach();
            return true;
        }

        public void SetTransportMatrix(Matrix4x4 transportAtoB)
        {
            transportMatrix = SourceNode.IsZeroMatrix(transportAtoB) ? Matrix4x4.identity : transportAtoB;
        }

        public bool Connects(SourceNode node)
        {
            return node != null && (node == endpointA || node == endpointB);
        }

        public SourceNode Other(SourceNode node)
        {
            if (node == endpointA)
                return endpointB;
            if (node == endpointB)
                return endpointA;
            return null;
        }

        public bool TryGetTransportMatrix(SourceNode from, out Matrix4x4 matrix)
        {
            EnsureTransport();
            if (from == endpointA)
            {
                matrix = Effective(endpointA, endpointB, transportMatrix);
                return true;
            }

            if (from == endpointB && TryInvert(transportMatrix, out var reverseRest))
            {
                matrix = Effective(endpointB, endpointA, reverseRest);
                return true;
            }

            matrix = Matrix4x4.identity;
            return false;
        }

        /// <summary>
        /// 出発側の位相は XY 回転、到着側の位相は XZ 回転。平面が違うので順序を入れ替えると残る。
        /// </summary>
        public static Matrix4x4 SpinCoupling(SourceNode from, SourceNode to)
        {
            if (from == null || to == null)
                return Matrix4x4.identity;

            return RotationXZ(to.Phase) * RotationXY(from.Phase);
        }

        static Matrix4x4 Effective(SourceNode from, SourceNode to, Matrix4x4 rest)
        {
            return SpinCoupling(from, to) * rest;
        }

        /// <summary>from の枠にある値へ、その向きの補正行列を掛ける。</summary>
        public bool TryTransport(SourceNode from, Vector4 value, out Vector4 transported)
        {
            if (!TryGetTransportMatrix(from, out var matrix))
            {
                transported = value;
                return false;
            }

            transported = matrix * value;
            return true;
        }

        /// <summary>from の状態を補正して相手ノードの受信箱へ届ける。</summary>
        public bool Deliver(SourceNode from, Vector4 value)
        {
            if (!TryTransport(from, value, out var transported))
                return false;

            var to = Other(from);
            if (to == null)
                return false;

            to.Receive(transported, this, from);
            return true;
        }

        /// <summary>
        /// origin から path の順にリンクを渡り、各ホップでその向きの補正行列を掛ける。
        /// 右の行列から先に作用する（経路順）。
        /// </summary>
        public static bool TryTransportAlong(SourceNode origin, Vector4 value, out Vector4 arrived, params UniversalLink[] path)
        {
            arrived = value;
            var cursor = origin;
            if (cursor == null || path == null)
                return false;

            for (var i = 0; i < path.Length; i++)
            {
                var link = path[i];
                if (link == null || !link.TryTransport(cursor, arrived, out arrived))
                    return false;

                cursor = link.Other(cursor);
                if (cursor == null)
                    return false;
            }

            return true;
        }

        public static Matrix4x4 RotationXY(float radians)
        {
            var cosine = Mathf.Cos(radians);
            var sine = Mathf.Sin(radians);
            var matrix = Matrix4x4.identity;
            matrix.m00 = cosine;
            matrix.m01 = -sine;
            matrix.m10 = sine;
            matrix.m11 = cosine;
            return matrix;
        }

        public static Matrix4x4 RotationXZ(float radians)
        {
            var cosine = Mathf.Cos(radians);
            var sine = Mathf.Sin(radians);
            var matrix = Matrix4x4.identity;
            matrix.m00 = cosine;
            matrix.m02 = -sine;
            matrix.m20 = sine;
            matrix.m22 = cosine;
            return matrix;
        }

        /// <summary>
        /// 同じ始点と終点を、A→B→C と A→D→C の2経路で運び、到着値の差をログとして返す。
        /// XY 回転と XZ 回転は非可換なので、経路順が違えば差分が残る。
        /// 全辺を同じ回転にそろえた対照では差分は残らない。
        /// </summary>
        [ContextMenu("非可換性を検証")]
        void RunNonCommutativityTest()
        {
            TestNonCommutativity();
        }

        public static NonCommutativityReport TestNonCommutativity()
        {
            return TestNonCommutativity(1e-4f);
        }

        public static NonCommutativityReport TestNonCommutativity(float gapEpsilon)
        {
            var owned = new List<GameObject>(8);
            try
            {
                var probe = new Vector4(1f, 0f, 0f, 0f);
                var nodeA = CreateNode(owned, "A", probe);
                var nodeB = CreateNode(owned, "B", Vector4.zero);
                var nodeC = CreateNode(owned, "C", Vector4.zero);
                var nodeD = CreateNode(owned, "D", Vector4.zero);

                var rotationXY = RotationXY(0.7f);
                var rotationXZ = RotationXZ(1.1f);

                var linkAB = CreateLink(owned, nodeA, nodeB, rotationXY);
                var linkBC = CreateLink(owned, nodeB, nodeC, rotationXZ);
                var linkAD = CreateLink(owned, nodeA, nodeD, rotationXZ);
                var linkDC = CreateLink(owned, nodeD, nodeC, rotationXY);

                var directionChanges = linkAB.TryTransport(nodeA, probe, out var towardB)
                    && linkAB.TryTransport(nodeB, probe, out var towardA)
                    && (towardB - towardA).sqrMagnitude > gapEpsilon * gapEpsilon;

                var pipeMatches = nodeA.Send(linkAB)
                    && nodeB.Inbox.Count == 1
                    && (nodeB.Inbox[0].Value - towardB).sqrMagnitude <= gapEpsilon * gapEpsilon
                    && nodeB.Inbox[0].From == nodeA
                    && nodeB.Inbox[0].ReceiverProperTime == nodeB.ProperTime;

                var abcOk = TryTransportAlong(nodeA, probe, out var alongAbc, linkAB, linkBC);
                var adcOk = TryTransportAlong(nodeA, probe, out var alongAdc, linkAD, linkDC);
                var composedAbc = rotationXZ * rotationXY * probe;
                var composedAdc = rotationXY * rotationXZ * probe;
                var walkerMatches = abcOk
                    && adcOk
                    && (alongAbc - composedAbc).sqrMagnitude <= gapEpsilon * gapEpsilon
                    && (alongAdc - composedAdc).sqrMagnitude <= gapEpsilon * gapEpsilon;

                linkAB.SetTransportMatrix(rotationXY);
                linkBC.SetTransportMatrix(rotationXY);
                linkAD.SetTransportMatrix(rotationXY);
                linkDC.SetTransportMatrix(rotationXY);

                var controlAbc = Vector4.zero;
                var controlAdc = Vector4.zero;
                var controlOk = TryTransportAlong(nodeA, probe, out controlAbc, linkAB, linkBC)
                    && TryTransportAlong(nodeA, probe, out controlAdc, linkAD, linkDC);
                var controlGap = controlOk ? (controlAbc - controlAdc).magnitude : float.NaN;
                var controlHolds = controlOk && controlGap <= gapEpsilon;

                var gap = Vector4.zero;
                var gapMagnitude = float.NaN;
                if (abcOk && adcOk)
                {
                    gap = alongAbc - alongAdc;
                    gapMagnitude = gap.magnitude;
                }

                var hasGap = directionChanges
                    && pipeMatches
                    && walkerMatches
                    && controlHolds
                    && gapMagnitude > gapEpsilon;

                var report = new NonCommutativityReport(
                    alongAbc,
                    alongAdc,
                    gap,
                    gapMagnitude,
                    hasGap,
                    BuildLog(directionChanges, pipeMatches, walkerMatches, controlHolds, alongAbc, alongAdc, gap, gapMagnitude, controlGap, hasGap));

                if (hasGap)
                    Debug.Log(report.Log);
                else
                    Debug.LogWarning(report.Log);

                return report;
            }
            finally
            {
                for (var i = owned.Count - 1; i >= 0; i--)
                    DestroyOwned(owned[i]);
            }
        }

        void Attach()
        {
            if (attachedA == endpointA && attachedB == endpointB)
                return;

            Detach();
            attachedA = endpointA;
            attachedB = endpointB;
            attachedA?.RegisterLink(this);
            attachedB?.RegisterLink(this);
        }

        void Detach()
        {
            if (attachedA != null)
                attachedA.UnregisterLink(this);
            if (attachedB != null)
                attachedB.UnregisterLink(this);

            attachedA = null;
            attachedB = null;
        }

        void EnsureTransport()
        {
            if (SourceNode.IsZeroMatrix(transportMatrix))
                transportMatrix = Matrix4x4.identity;
        }

        static bool TryInvert(Matrix4x4 matrix, out Matrix4x4 inverse)
        {
            if (Mathf.Abs(matrix.determinant) < SingularDeterminant)
            {
                inverse = Matrix4x4.identity;
                return false;
            }

            inverse = matrix.inverse;
            return true;
        }

        static SourceNode CreateNode(List<GameObject> owned, string name, Vector4 state)
        {
            var owner = new GameObject(name);
            owner.hideFlags = HideFlags.HideAndDontSave;
            owned.Add(owner);
            var node = owner.AddComponent<SourceNode>();
            node.SetState(state);
            return node;
        }

        static UniversalLink CreateLink(List<GameObject> owned, SourceNode a, SourceNode b, Matrix4x4 transportAtoB)
        {
            var owner = new GameObject("Link " + a.name + "-" + b.name);
            owner.hideFlags = HideFlags.HideAndDontSave;
            owned.Add(owner);
            var link = owner.AddComponent<UniversalLink>();
            link.Bind(a, b, transportAtoB);
            return link;
        }

        static void DestroyOwned(GameObject owner)
        {
            if (owner == null)
                return;

            if (Application.isPlaying)
                Destroy(owner);
            else
                DestroyImmediate(owner);
        }

        static string BuildLog(
            bool directionChanges,
            bool pipeMatches,
            bool walkerMatches,
            bool controlHolds,
            Vector4 alongAbc,
            Vector4 alongAdc,
            Vector4 gap,
            float gapMagnitude,
            float controlGap,
            bool hasGap)
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "[UniversalLink] 非可換性ログ\n" +
                "向き A→B と B→A で値が変わる: {0}\n" +
                "A から B への送受信: {1}\n" +
                "経路 A→B→C: {2}\n" +
                "経路 A→D→C: {3}\n" +
                "到着値の差分: {4}\n" +
                "差分の大きさ: {5:F5}\n" +
                "可換な割り当てでの差分の大きさ: {6:F5}\n" +
                "経路順の合成と転送結果: {7}\n" +
                "可換コントロール: {8}\n" +
                "非可換なズレ: {9}\n" +
                "この差分は接続を調整するためのログとして返す。",
                directionChanges ? "確認" : "未確認",
                pipeMatches ? "受信値が補正結果と一致" : "不一致",
                FormatVector(alongAbc),
                FormatVector(alongAdc),
                FormatVector(gap),
                gapMagnitude,
                controlGap,
                walkerMatches ? "一致" : "不一致",
                controlHolds ? "差分なし" : "不一致",
                hasGap ? "検出" : "未検出");
        }

        static string FormatVector(Vector4 value)
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "({0:F4}, {1:F4}, {2:F4}, {3:F4})",
                value.x,
                value.y,
                value.z,
                value.w);
        }

        /// <summary>2経路の到着値と、その差。差が残ったこと自体がこの検証の記録。</summary>
        public readonly struct NonCommutativityReport
        {
            public NonCommutativityReport(
                Vector4 alongAbc,
                Vector4 alongAdc,
                Vector4 gap,
                float gapMagnitude,
                bool hasNonCommutativeGap,
                string log)
            {
                AlongAbc = alongAbc;
                AlongAdc = alongAdc;
                Gap = gap;
                GapMagnitude = gapMagnitude;
                HasNonCommutativeGap = hasNonCommutativeGap;
                Log = log;
            }

            public Vector4 AlongAbc { get; }
            public Vector4 AlongAdc { get; }
            public Vector4 Gap { get; }
            public float GapMagnitude { get; }
            public bool HasNonCommutativeGap { get; }
            public string Log { get; }

            public override string ToString()
            {
                return Log;
            }
        }
    }
}
