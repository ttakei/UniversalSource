using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace UniversalSource
{
    /// <summary>
    /// TransportSample シーンの再生時に、ノードの固有時間とリンクの転送を確認する。
    /// オブジェクトの配置は画面上の目安であり、計算はリンクの補正行列だけを使う。
    /// </summary>
    [DisallowMultipleComponent]
    [AddComponentMenu("UniversalSource/Transport Sample Driver")]
    public sealed class TransportSampleDriver : MonoBehaviour
    {
        const float GapEpsilon = 1e-4f;

        [SerializeField] SourceNode nodeA = null;
        [SerializeField] SourceNode nodeB = null;
        [SerializeField] SourceNode nodeC = null;
        [SerializeField] SourceNode nodeD = null;
        [SerializeField] UniversalLink linkAB = null;
        [SerializeField] UniversalLink linkBC = null;
        [SerializeField] UniversalLink linkAD = null;
        [SerializeField] UniversalLink linkDC = null;

        string report = "";
        bool passed;
        Vector2 scroll;
        GUIStyle bodyStyle;

        void Start()
        {
            passed = RunChecks();
            if (passed)
                Debug.Log(report);
            else
                Debug.LogWarning(report);
        }

        void OnGUI()
        {
            if (bodyStyle == null)
            {
                bodyStyle = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 15,
                    wordWrap = true,
                    richText = false
                };
                bodyStyle.normal.textColor = new Color(0.1f, 0.12f, 0.14f);
            }

            var area = new Rect(16f, 16f, Mathf.Min(860f, Screen.width - 32f), Mathf.Min(560f, Screen.height - 32f));
            GUILayout.BeginArea(area, GUI.skin.box);
            if (GUILayout.Button("確認を再実行"))
            {
                passed = RunChecks();
                if (passed)
                    Debug.Log(report);
                else
                    Debug.LogWarning(report);
            }

            scroll = GUILayout.BeginScrollView(scroll);
            GUILayout.Label(string.IsNullOrEmpty(report) ? "確認待ち" : report, bodyStyle);
            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }

        public bool RunChecks()
        {
            var lines = new List<string>();
            var culture = CultureInfo.InvariantCulture;

            if (!HasReferences())
            {
                report = "結果: 未通過。ノードとリンクの参照を確認する。";
                passed = false;
                return false;
            }

            var wired = linkAB.Bind(nodeA, nodeB, linkAB.TransportMatrix)
                && linkBC.Bind(nodeB, nodeC, linkBC.TransportMatrix)
                && linkAD.Bind(nodeA, nodeD, linkAD.TransportMatrix)
                && linkDC.Bind(nodeD, nodeC, linkDC.TransportMatrix)
                && nodeA.LinkTo(nodeB) == linkAB
                && nodeB.LinkTo(nodeC) == linkBC
                && nodeA.LinkTo(nodeD) == linkAD
                && nodeD.LinkTo(nodeC) == linkDC;

            var stateA = nodeA.State;
            var stateB = nodeB.State;
            var timeA = nodeA.ProperTime;
            var timeB = nodeB.ProperTime;
            var timeC = nodeC.ProperTime;
            var timeD = nodeD.ProperTime;

            nodeA.Step();
            nodeB.Step();
            nodeB.Step();

            var clocksOk = nodeA.ProperTime == timeA + 1
                && nodeB.ProperTime == timeB + 2
                && nodeC.ProperTime == timeC
                && nodeD.ProperTime == timeD;
            var stateAMoved = (nodeA.State - stateA).sqrMagnitude > 1e-8f;
            var stateBHeld = (nodeB.State - stateB).sqrMagnitude <= 1e-8f;

            var probe = nodeA.State;
            var sent = nodeA.Send(linkAB);
            var transportOk = linkAB.TryTransport(nodeA, probe, out var transported);
            var received = Vector4.zero;
            var receivedTime = 0;
            var pipeMatches = false;
            if (sent && transportOk && nodeB.Inbox.Count > 0)
            {
                var signal = nodeB.Inbox[nodeB.Inbox.Count - 1];
                received = signal.Value;
                receivedTime = signal.ReceiverProperTime;
                pipeMatches = (received - transported).sqrMagnitude <= GapEpsilon * GapEpsilon
                    && signal.From == nodeA
                    && signal.ReceiverProperTime == nodeB.ProperTime;
            }

            var reversed = Vector4.zero;
            var directionOk = transportOk
                && linkAB.TryTransport(nodeB, probe, out reversed)
                && (transported - reversed).sqrMagnitude > GapEpsilon * GapEpsilon;

            var abcOk = UniversalLink.TryTransportAlong(nodeA, probe, out var alongAbc, linkAB, linkBC);
            var adcOk = UniversalLink.TryTransportAlong(nodeA, probe, out var alongAdc, linkAD, linkDC);
            var gap = alongAbc - alongAdc;
            var gapMagnitude = abcOk && adcOk ? gap.magnitude : float.NaN;
            var gapOk = abcOk && adcOk && gapMagnitude > GapEpsilon;

            var builtin = UniversalLink.TestNonCommutativity();
            var builtinOk = builtin.HasNonCommutativeGap;

            var ok = wired && clocksOk && stateAMoved && stateBHeld && pipeMatches && directionOk && gapOk && builtinOk;

            lines.Add(ok
                ? "結果: 通過。固有時間、送受信、向き、経路差を確認した。"
                : "結果: 未通過。各項目の数値を確認する。");
            lines.Add("配置座標は表示用。計算は各リンクの補正行列だけを使う。");
            lines.Add("");
            lines.Add(wired
                ? "接続: A-B、B-C、A-D、D-C が両端の近傍に登録されている。"
                : "接続: 近傍への登録を確認できなかった。");
            lines.Add(clocksOk && stateAMoved && stateBHeld
                ? string.Format(
                    culture,
                    "固有時間: A {0}→{1}、B {2}→{3}、C {4}、D {5}。Aの状態は {6} から {7} へ進み、Bの状態は {8} のまま。",
                    timeA,
                    nodeA.ProperTime,
                    timeB,
                    nodeB.ProperTime,
                    nodeC.ProperTime,
                    nodeD.ProperTime,
                    FormatVector(stateA),
                    FormatVector(nodeA.State),
                    FormatVector(nodeB.State))
                : string.Format(
                    culture,
                    "固有時間: A {0}、B {1}、C {2}、D {3}。状態 A {4}、B {5}。",
                    nodeA.ProperTime,
                    nodeB.ProperTime,
                    nodeC.ProperTime,
                    nodeD.ProperTime,
                    FormatVector(nodeA.State),
                    FormatVector(nodeB.State)));
            lines.Add(pipeMatches
                ? string.Format(
                    culture,
                    "送受信: A→B の受信値 {0}。受信時のBの固有時間は {1}。",
                    FormatVector(received),
                    receivedTime)
                : "送受信: A→B の受信値を確認できなかった。");
            lines.Add(directionOk
                ? string.Format(
                    culture,
                    "向き: A→B は {0}、B→A は {1}。",
                    FormatVector(transported),
                    FormatVector(reversed))
                : "向き: 逆方向の補正を確認できなかった。");
            lines.Add(gapOk
                ? string.Format(
                    culture,
                    "経路差: A→B→C {0}、A→D→C {1}、差分の大きさ {2:F5}。",
                    FormatVector(alongAbc),
                    FormatVector(alongAdc),
                    gapMagnitude)
                : "経路差: 2経路の到着値を確認できなかった。");
            lines.Add(builtinOk
                ? string.Format(culture, "組み込みテスト: 非可換なズレを検出。差分の大きさ {0:F5}。", builtin.GapMagnitude)
                : "組み込みテスト: 非可換なズレは未検出。");

            report = string.Join("\n", lines);
            passed = ok;
            return ok;
        }

        bool HasReferences()
        {
            return nodeA != null
                && nodeB != null
                && nodeC != null
                && nodeD != null
                && linkAB != null
                && linkBC != null
                && linkAD != null
                && linkDC != null;
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
    }
}
