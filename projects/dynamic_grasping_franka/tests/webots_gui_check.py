"""
End-to-end test of the lesson window: drives the real robot window, served by a
running Webots instance, through headless Chrome (DevTools protocol), clicking
every feature and checking what the page shows.  Used by run_webots_tests.sh.

    python3 webots_gui_check.py <screenshot_dir> <webots_port>

Needs: pip install websockets, and Chrome started with --remote-debugging-port=9333.
"""
import asyncio, json, base64, sys, urllib.request
import websockets

PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 1253
URL = f"http://localhost:{PORT}/robot_windows/grasping_window/grasping_window.html?name=panda"
OUT = sys.argv[1]
errors, console = [], []
results = []

def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(("PASS " if ok else "FAIL ") + name + (f"  [{detail}]" if detail else ""), flush=True)

async def main():
    tabs = json.load(urllib.request.urlopen("http://127.0.0.1:9333/json"))
    page = [t for t in tabs if t["type"] == "page"][0]
    async with websockets.connect(page["webSocketDebuggerUrl"], max_size=50_000_000) as ws:
        mid = 0
        pending = {}
        async def reader():
            async for raw in ws:
                m = json.loads(raw)
                if "id" in m and m["id"] in pending:
                    pending.pop(m["id"]).set_result(m)
                elif m.get("method") == "Runtime.exceptionThrown":
                    errors.append(m["params"]["exceptionDetails"].get("exception", {}).get("description") or m["params"]["exceptionDetails"]["text"])
                elif m.get("method") == "Runtime.consoleAPICalled":
                    t = m["params"]["type"]; txt = " ".join(str(a.get("value", a.get("description"))) for a in m["params"]["args"])
                    console.append((t, txt))
                    if t == "error": errors.append(txt)
                elif m.get("method") == "Log.entryAdded":
                    e = m["params"]["entry"]
                    if e["level"] == "error": errors.append(e["text"] + " " + e.get("url", ""))
        asyncio.create_task(reader())
        async def cmd(method, **params):
            nonlocal mid
            mid += 1; fut = asyncio.get_event_loop().create_future(); pending[mid] = fut
            await ws.send(json.dumps({"id": mid, "method": method, "params": params}))
            return (await fut).get("result", {})
        async def js(expr):
            r = await cmd("Runtime.evaluate", expression=expr, returnByValue=True, awaitPromise=True)
            if "exceptionDetails" in r: errors.append("eval: " + str(r["exceptionDetails"]))
            return r.get("result", {}).get("value")
        async def wait_for(expr, timeout=60):
            # While the page is still loading, elements may not exist yet:
            # treat that as "not yet" instead of an error.
            guarded = f"(()=>{{try {{ return ({expr}); }} catch (e) {{ return null; }}}})()"
            for _ in range(int(timeout * 5)):
                v = await js(guarded)
                if v: return v
                await asyncio.sleep(0.2)
            return None
        async def shot(name):
            r = await cmd("Page.captureScreenshot", format="png")
            open(f"{OUT}/{name}.png", "wb").write(base64.b64decode(r["data"]))

        await cmd("Runtime.enable"); await cmd("Log.enable"); await cmd("Page.enable")
        await cmd("Emulation.setDeviceMetricsOverride", width=1200, height=900, deviceScaleFactor=1, mobile=False)
        await cmd("Page.navigate", url=URL)
        phase = "document.getElementById('lesson-phase').textContent"

        # 1. connection + initial state
        check("window connects to controller", await wait_for("document.getElementById('connection').textContent==='connected'", 30))
        check("controller reaches IDLE", await wait_for(f"{phase}==='IDLE'", 30))
        check("telemetry updates sim time", await wait_for("document.getElementById('sim-time').textContent!=='t = 0.00 s'", 10))
        fk = await wait_for("(()=>{const t=document.getElementById('fk-check').textContent; return t.includes('mm')? t : null})()", 10)
        check("FK check below 5 mm", fk and float(fk.split(':')[1].split('mm')[0]) < 5, fk)
        check("7 presets listed", await js("document.querySelectorAll('#preset option').length") == 7)
        check("grip-force hint shows N_min", "0.33 N" in (await js("document.getElementById('fmin-hint').textContent") or ""))

        # 2. every tab renders
        for tab in ["camera", "planner", "motion", "gripper", "ik", "exercises", "log", "lesson"]:
            await js(f"document.querySelector('#tabs button[data-tab=\"{tab}\"]').click()")
            await asyncio.sleep(0.4)
            check(f"tab '{tab}' visible", await js(f"document.getElementById('tab-{tab}').classList.contains('active')"))

        # 2b. camera tab sees the ball waiting at its start position
        await js("document.querySelector('#tabs button[data-tab=\"camera\"]').click()")
        info = await wait_for("(()=>{const t=document.getElementById('camera-info').textContent; return t.includes('camera error') && !t.includes('camera error–') ? t : null})()", 10)
        cam_err = None
        if info and "mm" in info.split("camera error")[1]:
            cam_err = float(info.split("camera error")[1].split("mm")[0])
        check("Camera tab: ball detected, error < 10 mm", cam_err is not None and cam_err < 10, f"{cam_err} mm")
        await shot("gui_camera")

        # 2c. perception switch
        await js("document.querySelector('input[name=perception][value=simulator]').click()")
        check("perception switch -> simulator", await wait_for("document.getElementById('log').textContent.includes('now come from the simulator')", 5))
        await js("document.querySelector('input[name=perception][value=camera]').click()")
        check("perception switch -> camera", await wait_for("document.getElementById('log').textContent.includes('now come from the camera')", 5))

        # 2d. exercises tab
        await js("document.querySelector('#tabs button[data-tab=\"exercises\"]').click()")
        await js("document.getElementById('check-exercises').click()")
        check("Exercises: checker runs (0 / 6 on blank templates)", await wait_for("document.getElementById('exercise-results').textContent.includes('0 / 6 exercises correct')", 10))
        await js("document.querySelector('#tabs button[data-tab=\"lesson\"]').click()")

        # 3. preset + launch + catch
        await js("{const s=document.getElementById('preset'); s.value='3'; s.dispatchEvent(new Event('change'))}")
        check("preset fills sliders", await js("document.getElementById('ball-heading').value") == "130")
        await js("document.getElementById('launch').click()")
        check("Launch button enabled at IDLE", await js("!document.getElementById('launch').disabled"))
        check("pipeline shows OBSERVE", await wait_for(f"{phase}==='OBSERVE'", 10))
        await js("document.querySelector('#tabs button[data-tab=\"planner\"]').click()")
        check("planner table filled with an accepted row", await wait_for("document.querySelectorAll('#candidates tr.accepted').length===1", 20))
        summary = await js("document.getElementById('plan-summary').textContent")
        check("plan summary shown", summary and "Meet the ball" in summary, summary[:80] if summary else "")
        await shot("gui_planner")
        await js("document.querySelector('#tabs button[data-tab=\"motion\"]').click()")
        await asyncio.sleep(1.0); await shot("gui_motion")
        await js("document.querySelector('#tabs button[data-tab=\"lesson\"]').click()")
        check("Launch button disabled while busy", await js("document.getElementById('launch').disabled"))
        check("catch completes: 'Ball caught'", await wait_for("document.getElementById('outcome').textContent.includes('caught')", 30))
        check("all pipeline steps green", await js("[...document.querySelectorAll('#pipeline li')].every(li=>li.classList.contains('done'))"))
        await js("document.querySelector('#tabs button[data-tab=\"gripper\"]').click()")
        await asyncio.sleep(0.5); await shot("gui_gripper")
        check("log has entries", (await js("document.querySelectorAll('#log li').length")) >= 5)

        # 3b. CSV export downloads
        import os, glob
        await cmd("Browser.setDownloadBehavior", behavior="allow", downloadPath=OUT)
        await js("document.querySelector('#tabs button[data-tab=\"log\"]').click()")
        await js("document.getElementById('export-telemetry').click()")
        await js("document.getElementById('export-plan').click()")
        for _ in range(25):
            if os.path.exists(f"{OUT}/grasping_run.csv") and os.path.exists(f"{OUT}/planner_candidates.csv"): break
            await asyncio.sleep(0.2)
        run_rows = open(f"{OUT}/grasping_run.csv").read().splitlines() if os.path.exists(f"{OUT}/grasping_run.csv") else []
        plan_rows = open(f"{OUT}/planner_candidates.csv").read().splitlines() if os.path.exists(f"{OUT}/planner_candidates.csv") else []
        check("CSV export: run data", len(run_rows) > 50 and run_rows[0].startswith("t,phase,q1"), f"{len(run_rows)} rows")
        check("CSV export: planner candidates", len(plan_rows) > 5 and "accepted" in plan_rows[-1], f"{len(plan_rows)} rows")
        await js("document.querySelector('#tabs button[data-tab=\"lesson\"]').click()")

        # 4. reset
        await js("document.getElementById('reset').click()")
        check("reset returns to IDLE", await wait_for(f"{phase}==='IDLE'", 20))

        # 5. teaching mode through every pause
        await js("{const t=document.getElementById('teach'); t.click()}")
        await js("{const s=document.getElementById('preset'); s.value='2'; s.dispatchEvent(new Event('change'))}")
        await js("document.getElementById('launch').click()")
        pauses = []
        for _ in range(6):
            got = await wait_for(f"(()=>{{const p={phase}; return (p.startsWith('PAUSED') && !document.getElementById('next').disabled) || p==='CAUGHT' ? p : null}})()", 30)
            if not got or got == "CAUGHT": break
            pauses.append(got.split('next: ')[1])
            if len(pauses) == 1:
                check("quiz shown for the coming phase", await js("!document.getElementById('quiz').classList.contains('hidden')"))
                await js("document.querySelectorAll('#quiz-options button')[1].click()")
                check("quiz: right answer gets 'Correct'", "Correct" in (await js("document.getElementById('quiz-feedback').textContent") or ""))
                await shot("gui_paused")
            await asyncio.sleep(0.7)
            await js("document.getElementById('next').click()")
            await wait_for(f"{phase}!=={json.dumps(got)}", 10)
        check("teaching mode paused at PLAN, APPROACH, DESCEND, WAIT, LIFT", pauses == ["PLAN", "APPROACH", "DESCEND", "WAIT", "LIFT"], str(pauses))
        check("teaching-mode catch completes", await wait_for("document.getElementById('outcome').textContent.includes('caught')", 30))
        await js("document.getElementById('teach').click()")
        await js("document.getElementById('reset').click()")
        check("reset after teaching run", await wait_for(f"{phase}==='IDLE'", 20))

        # 6. IK lab
        await js("document.querySelector('#tabs button[data-tab=\"ik\"]').click()")
        await js("{for (const [id,v] of [['ik-x',0.4],['ik-y',-0.3],['ik-z',0.35],['ik-yaw',45]]) {const e=document.getElementById(id); e.value=v; e.dispatchEvent(new Event('input'));}}")
        await js("document.getElementById('ik-go').click()")
        check("IK lab: solved", await wait_for("document.getElementById('ik-result').textContent.includes('solved')", 10))
        check("IK lab: arm moves (IK_MOVE) and returns to IDLE", await wait_for(f"{phase}==='IDLE'", 20))
        await asyncio.sleep(0.5); await shot("gui_ik")
        await js("{for (const [id,v] of [['ik-x',0.8],['ik-y',0.8],['ik-z',0.03]]) {const e=document.getElementById(id); e.value=v; e.dispatchEvent(new Event('input'));}}")
        await js("document.getElementById('ik-go').click()")
        check("IK lab: unreachable goal reported", await wait_for("document.getElementById('ik-result').textContent.includes('no solution')", 10))

        # 7. grip force via slider -> drop
        await js("document.getElementById('reset').click()"); await wait_for(f"{phase}==='GO_HOME'", 5); await wait_for(f"{phase}==='IDLE'", 20)
        await js("{const g=document.getElementById('grip-force'); g.value=0.2; g.dispatchEvent(new Event('input')); g.dispatchEvent(new Event('change'))}")
        check("hint warns when grip < N_min", await js("document.getElementById('fmin-hint').classList.contains('warn')"))
        await js("{const s=document.getElementById('preset'); s.value='0'; s.dispatchEvent(new Event('change'))}")
        await js("document.getElementById('launch').click()")
        check("low grip force -> 'Dropped'", await wait_for("document.getElementById('outcome').textContent.includes('Dropped')", 30))
        await js("{const g=document.getElementById('grip-force'); g.value=20; g.dispatchEvent(new Event('input')); g.dispatchEvent(new Event('change'))}")

        # 8. out of reach
        await js("document.getElementById('reset').click()"); await wait_for(f"{phase}==='GO_HOME'", 5); await wait_for(f"{phase}==='IDLE'", 20)
        await js("{const s=document.getElementById('preset'); s.value='6'; s.dispatchEvent(new Event('change'))}")
        await js("document.getElementById('launch').click()")
        check("out-of-reach preset -> 'No feasible intercept'", await wait_for("document.getElementById('outcome').textContent.includes('No feasible')", 20))
        await shot("gui_noplan")

        # 9. reload the page mid-session: reconnects and gets state
        await cmd("Page.reload")
        check("page reload reconnects", await wait_for("document.getElementById('connection').textContent==='connected'", 20))
        check("reloaded page shows current phase", await wait_for(f"{phase}==='NO_PLAN'", 10))

        await asyncio.sleep(0.5)
        check("no JavaScript errors", not errors, "; ".join(errors)[:300])
        print(f"SUMMARY {sum(r[1] for r in results)}/{len(results)} passed", flush=True)

asyncio.run(main())
