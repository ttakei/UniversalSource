using System.Collections.Generic;
using UnityEngine;

namespace UniversalSource
{
    /// <summary>
    /// 絶対座標を持たない観測者ノード。
    /// 内部状態と固有時間（局所ステップ）だけを持ち、周囲の <see cref="UniversalLink"/> でのみ他ノードと通信する。
    /// </summary>
    [DisallowMultipleComponent]
    [AddComponentMenu("UniversalSource/Source Node")]
    public sealed class SourceNode : MonoBehaviour
    {
        public const int InboxCapacity = 8;

        [SerializeField] Vector4 state = new Vector4(1f, 0f, 0f, 0f);
        [SerializeField] Matrix4x4 localEvolution = Matrix4x4.identity;
        [SerializeField] int properTime;
        [SerializeField] float phase;
        [SerializeField] float spinRate;
        [SerializeField] float spinAmplitude;

        float phaseAnchor;
        int spinClock;
        bool phaseAnchorReady;

        readonly List<UniversalLink> links = new List<UniversalLink>(4);
        readonly List<ReceivedSignal> inbox = new List<ReceivedSignal>(InboxCapacity);

        public Vector4 State => state;

        /// <summary>このノードが自ら進めた離散ステップ数。共有時計ではない。</summary>
        public int ProperTime => properTime;

        /// <summary>内部スピンの位相角（ラジアン）。リンクの補正行列がこれを読む。</summary>
        public float Phase => phase;

        /// <summary>位相振動の角速度。このノードの歩みだけで進む。</summary>
        public float SpinRate => spinRate;

        public float SpinAmplitude => spinAmplitude;

        public Matrix4x4 LocalEvolution => localEvolution;
        public IReadOnlyList<UniversalLink> Links => links;
        public IReadOnlyList<ReceivedSignal> Inbox => inbox;

        void Reset()
        {
            state = new Vector4(1f, 0f, 0f, 0f);
            localEvolution = Matrix4x4.identity;
            properTime = 0;
            phase = 0f;
            spinRate = 0f;
            spinAmplitude = 0f;
        }

        void Awake()
        {
            if (IsZeroMatrix(localEvolution))
                localEvolution = Matrix4x4.identity;
            EnsurePhaseAnchor();
        }

        public void SetState(Vector4 value)
        {
            state = value;
        }

        public void SetLocalEvolution(Matrix4x4 evolution)
        {
            localEvolution = IsZeroMatrix(evolution) ? Matrix4x4.identity : evolution;
        }

        public void SetPhase(float radians)
        {
            phase = WrapPhase(radians);
            phaseAnchor = phase;
            spinClock = 0;
            phaseAnchorReady = true;
        }

        public void SetSpin(float rate, float amplitude)
        {
            spinRate = rate;
            spinAmplitude = amplitude;
        }

        /// <summary>
        /// 位相をこのノードのスピンだけ進める。基準角のまわりを spinAmplitude で揺れる。
        /// </summary>
        public void AdvancePhase()
        {
            EnsurePhaseAnchor();
            spinClock++;
            phase = WrapPhase(phaseAnchor + spinAmplitude * Mathf.Sin(spinClock * spinRate));
        }

        /// <summary>
        /// このノード自身の時計を 1 ステップ進める。状態にはノード固有の発展行列だけを掛ける。
        /// </summary>
        [ContextMenu("固有時間を1ステップ進める")]
        public void Step()
        {
            if (IsZeroMatrix(localEvolution))
                localEvolution = Matrix4x4.identity;

            state = localEvolution * state;
            AdvancePhase();
            properTime++;
        }

        internal void RegisterLink(UniversalLink link)
        {
            if (link == null || links.Contains(link) || !link.Connects(this))
                return;

            links.Add(link);
        }

        internal void UnregisterLink(UniversalLink link)
        {
            links.Remove(link);
        }

        public UniversalLink LinkTo(SourceNode other)
        {
            for (var i = 0; i < links.Count; i++)
            {
                if (links[i].Other(this) == other)
                    return links[i];
            }

            return null;
        }

        /// <summary>現在の状態を、指定リンクの補正行列を通して相手の枠へ送る。</summary>
        public bool Send(UniversalLink link)
        {
            if (link == null || !links.Contains(link))
                return false;

            return link.Deliver(this, state);
        }

        public bool SendTo(SourceNode neighbor)
        {
            return Send(LinkTo(neighbor));
        }

        /// <summary>周囲に登録されている全リンクへ、現在の状態を送る。</summary>
        [ContextMenu("周囲へ送信")]
        public int SendToNeighbors()
        {
            var sent = 0;
            for (var i = 0; i < links.Count; i++)
            {
                if (Send(links[i]))
                    sent++;
            }

            return sent;
        }

        /// <summary>
        /// 補正行列を適用済みの値を、このノードの枠で受け取る。
        /// 受信時刻は相手の時計ではなく、このノードの固有時間で記録する。
        /// </summary>
        public void Receive(Vector4 valueInLocalFrame, UniversalLink via, SourceNode from)
        {
            if (via == null || from == null || from == this)
                return;
            if (!via.Connects(this) || via.Other(from) != this)
                return;

            if (inbox.Count == InboxCapacity)
                inbox.RemoveAt(0);

            inbox.Add(new ReceivedSignal(valueInLocalFrame, via, from, properTime));
        }

        public void ClearInbox()
        {
            inbox.Clear();
        }

        void EnsurePhaseAnchor()
        {
            if (phaseAnchorReady)
                return;

            phaseAnchor = phase;
            phaseAnchorReady = true;
        }

        public static float WrapPhase(float radians)
        {
            const float twoPi = Mathf.PI * 2f;
            return Mathf.Repeat(radians + Mathf.PI, twoPi) - Mathf.PI;
        }

        internal static bool IsZeroMatrix(Matrix4x4 matrix)
        {
            for (var column = 0; column < 4; column++)
            {
                var value = matrix.GetColumn(column);
                if (value.x != 0f || value.y != 0f || value.z != 0f || value.w != 0f)
                    return false;
            }

            return true;
        }

        /// <summary>受信側の枠へ運ばれた値。時刻は受信ノードの固有時間。</summary>
        public readonly struct ReceivedSignal
        {
            public ReceivedSignal(Vector4 value, UniversalLink via, SourceNode from, int receiverProperTime)
            {
                Value = value;
                Via = via;
                From = from;
                ReceiverProperTime = receiverProperTime;
            }

            public Vector4 Value { get; }
            public UniversalLink Via { get; }
            public SourceNode From { get; }
            public int ReceiverProperTime { get; }
        }
    }
}
