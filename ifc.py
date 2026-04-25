import json
import csv
import math
import ifcopenshell
import ifcopenshell.api
from shapely.geometry import Polygon, MultiPolygon, LineString, Point
from shapely.ops import unary_union

class BIMPipeline:
    def __init__(self, scale=50.0, default_wall_height=3000.0):
        self.scale = scale
        self.height = default_wall_height
        self.mapping = self.load_mapping('mapping.json')

    def load_mapping(self, json_path):
        mapping = {}
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                mapping = json.load(f)
        except Exception as e:
            print(f"Warning: Failed to load mapping from {json_path}: {e}")
        return mapping

    def normalize_label(self, label):
        """STEP 1: Normalize label to ensure consistent mapping matching."""
        if not label:
            return ""
        return str(label).strip().lower().replace(" ", "")

    def get_coords(self, vertices, canvas_h):
        return [(v[0] * self.scale, (canvas_h - v[1]) * self.scale) for v in vertices]

    def get_centerline_and_thickness(self, polygon):
        """STEP 6: Kept intact - Finds centerline and thickness."""
        rect = polygon.minimum_rotated_rectangle
        coords = list(rect.exterior.coords)
        
        edges = []
        for i in range(4):
            p1 = coords[i]
            p2 = coords[i+1]
            length = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            edges.append((length, p1, p2))
            
        edges.sort(key=lambda x: x[0])
        thickness = edges[0][0]
        
        short1, short2 = edges[0], edges[1]
        mid1 = ((short1[1][0] + short1[2][0])/2, (short1[1][1] + short1[2][1])/2)
        mid2 = ((short2[1][0] + short2[2][0])/2, (short2[1][1] + short2[2][1])/2)
        
        dx = mid2[0] - mid1[0]
        dy = mid2[1] - mid1[1]
        angle = math.degrees(math.atan2(dy, dx)) % 180
        
        return LineString([mid1, mid2]), thickness, angle

    def preprocess_walls(self, wall_elements, canvas_h):
        """STEP 6: Kept intact - Wall merging logic."""
        groups = {}
        for e in wall_elements:
            vertices = self.get_coords(e['vertices'], canvas_h)
            poly = Polygon(vertices)
            cline, thickness, angle = self.get_centerline_and_thickness(poly)
            
            r_thickness = round(thickness, 1)
            r_angle = round(angle) % 180
            if r_angle == 180: r_angle = 0
            
            rad = math.radians(r_angle)
            nx, ny = -math.sin(rad), math.cos(rad)
            mid = cline.interpolate(0.5, normalized=True)
            offset = round(mid.x * nx + mid.y * ny, 0)
            
            group_key = (r_thickness, r_angle, offset)
            groups.setdefault(group_key, []).append((cline, poly))
            
        merged_walls_data = []
        for key, walls in groups.items():
            thickness, angle, offset = key
            
            rad = math.radians(angle)
            dx, dy = math.cos(rad), math.sin(rad)
            
            intervals = []
            for cline, poly in walls:
                p1, p2 = cline.coords[0], cline.coords[1]
                t1 = p1[0]*dx + p1[1]*dy
                t2 = p2[0]*dx + p2[1]*dy
                intervals.append((min(t1, t2), max(t1, t2), poly))
                
            intervals.sort(key=lambda x: x[0])
            if not intervals: continue
            
            merged_groups = []
            curr_end = intervals[0][1]
            curr_polys = [intervals[0][2]]
            
            for i in range(1, len(intervals)):
                start, end, poly = intervals[i]
                if start <= curr_end + 1.0:
                    curr_end = max(curr_end, end)
                    curr_polys.append(poly)
                else:
                    merged_groups.append(curr_polys)
                    curr_end = end
                    curr_polys = [poly]
            merged_groups.append(curr_polys)
            
            for polys in merged_groups:
                merged_poly = unary_union(polys)
                if isinstance(merged_poly, Polygon):
                    merged_walls_data.append({"geom": merged_poly, "thickness": thickness})
                else:
                    for geom in merged_poly.geoms:
                        if isinstance(geom, Polygon):
                            merged_walls_data.append({"geom": geom, "thickness": thickness})
                            
        return merged_walls_data

    def create_profile(self, model, poly):
        ext_coords = list(poly.exterior.coords)[:-1]
        ext_pts = [model.create_entity("IfcCartesianPoint", Coordinates=(float(c[0]), float(c[1]))) for c in ext_coords]
        ext_curve = model.create_entity("IfcPolyline", Points=ext_pts)
        
        if len(poly.interiors) == 0:
            return model.create_entity("IfcArbitraryClosedProfileDef", ProfileType="AREA", OuterCurve=ext_curve)
        
        inner_curves = []
        for interior in poly.interiors:
            int_coords = list(interior.coords)[:-1]
            int_pts = [model.create_entity("IfcCartesianPoint", Coordinates=(float(c[0]), float(c[1]))) for c in int_coords]
            inner_curves.append(model.create_entity("IfcPolyline", Points=int_pts))
        return model.create_entity("IfcArbitraryProfileDefWithVoids", ProfileType="AREA", OuterCurve=ext_curve, InnerCurves=inner_curves)

    def create_extrusion(self, model, profile, z_elevation, z_height):
        origin = model.create_entity("IfcAxis2Placement3D", Location=model.create_entity("IfcCartesianPoint", Coordinates=(0., 0., float(z_elevation))))
        return model.create_entity("IfcExtrudedAreaSolid", SweptArea=profile, Position=origin, 
                                   ExtrudedDirection=model.create_entity("IfcDirection", DirectionRatios=(0.,0.,1.)), 
                                   Depth=float(z_height))

    def run(self, json_path, output_path):
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        all_objs = data.get('elements', []) + data.get('rooms', [])
        if not all_objs:
            print("No elements found in JSON.")
            return
            
        canvas_h = max(v[1] for e in all_objs for v in e['vertices'])

        model = ifcopenshell.file(schema="IFC4")
        project = ifcopenshell.api.run("root.create_entity", model, ifc_class="IfcProject", name="BIM Project")
        ifcopenshell.api.run("context.add_context", model, context_type="Model")
        ifcopenshell.api.run("unit.assign_unit", model)
        
        site = ifcopenshell.api.run("root.create_entity", model, ifc_class="IfcSite", name="Site")
        building = ifcopenshell.api.run("root.create_entity", model, ifc_class="IfcBuilding", name="Building")
        storey = ifcopenshell.api.run("root.create_entity", model, ifc_class="IfcBuildingStorey", name="L0")

        ifcopenshell.api.run("aggregate.assign_object", model, products=[site], relating_object=project)
        ifcopenshell.api.run("aggregate.assign_object", model, products=[building], relating_object=site)
        ifcopenshell.api.run("aggregate.assign_object", model, products=[storey], relating_object=building)

        contexts = model.by_type("IfcGeometricRepresentationContext")
        context = next((c for c in contexts if c.ContextType == "Model"), contexts[0])

        # 2. PROCESS WALLS
        wall_elements = []
        other_elements = []
        
        # STEP 2: Fix mapping usage for elements
        for e in data.get('elements', []):
            label_raw = e.get('label', '')
            label = self.normalize_label(label_raw)
            map_info = self.mapping.get(label)
            
            if not map_info:
                print(f"[WARN] Mapping not found for element: '{label_raw}'")
                continue
                
            ifc_class = map_info.get('IfcEntity')
            print(f"[INFO] Processing element: {label_raw} | IFC class: {ifc_class}")
            
            if ifc_class == 'IfcWall':
                wall_elements.append(e)
            else:
                other_elements.append((e, map_info))

        merged_walls = self.preprocess_walls(wall_elements, canvas_h)
        
        created_walls = []
        for i, wall_data in enumerate(merged_walls):
            poly = wall_data["geom"]
            wall_entity = ifcopenshell.api.run("root.create_entity", model, ifc_class="IfcWall", name=f"Wall_{i}")
            ifcopenshell.api.run("spatial.assign_container", model, products=[wall_entity], relating_structure=storey)
            
            profile = self.create_profile(model, poly)
            solid = self.create_extrusion(model, profile, z_elevation=0.0, z_height=self.height)
            
            shape_rep = model.create_entity("IfcShapeRepresentation", ContextOfItems=context, RepresentationIdentifier="Body", RepresentationType="SweptSolid", Items=[solid])
            wall_entity.Representation = model.create_entity("IfcProductDefinitionShape", Representations=[shape_rep])
            wall_entity.ObjectPlacement = model.create_entity("IfcLocalPlacement", PlacementRelTo=storey.ObjectPlacement, RelativePlacement=model.create_entity("IfcAxis2Placement3D", Location=model.create_entity("IfcCartesianPoint", Coordinates=(0.,0.,0.))))
            
            created_walls.append({"entity": wall_entity, "geom": poly})

        # 3. PROCESS NON-WALL ELEMENTS (Windows/Doors/Etc)
        # STEP 3: Replace hardcoded window/door logic
        for e, map_info in other_elements:
            ifc_class = map_info.get('IfcEntity')
            
            if ifc_class in ['IfcWindow', 'IfcDoor']:
                item_poly = Polygon(self.get_coords(e['vertices'], canvas_h))
                
                # STEP 4: Improve opening-to-wall assignment
                best_wall = None
                
                # Try intersection first
                for wall_data in created_walls:
                    if wall_data["geom"].intersects(item_poly):
                        best_wall = wall_data
                        break
                
                # Fallback to nearest distance if no direct intersection
                if not best_wall:
                    min_dist = float('inf')
                    for wall_data in created_walls:
                        dist = wall_data["geom"].distance(item_poly)
                        if dist < min_dist:
                            min_dist = dist
                            best_wall = wall_data
                    
                    # Threshold for closest wall (e.g., 200 units/mm depending on scale)
                    threshold = 200.0 * (self.scale / 50.0)
                    if min_dist > threshold:
                        best_wall = None
                
                if not best_wall:
                    print(f"[WARN] Could not find a nearby wall for {ifc_class} ID {e.get('id')}. Skipping.")
                    continue
                
                # STEP 5: Separate Opening and Filling geometry
                is_window = (ifc_class == "IfcWindow")
                z_elev = 900.0 if is_window else 0.0
                z_height = 1200.0 if is_window else 2100.0
                
                name_attr = map_info.get('Name', e.get('label', 'Element'))
                
                # 1. Create Opening Element geometry (void)
                opening = ifcopenshell.api.run("root.create_entity", model, ifc_class="IfcOpeningElement", name=f"Opening_{e['id']}")
                opening_profile = self.create_profile(model, item_poly)
                opening_solid = self.create_extrusion(model, opening_profile, z_elev, z_height)
                
                opening_rep = model.create_entity("IfcShapeRepresentation", ContextOfItems=context, RepresentationIdentifier="Body", RepresentationType="SweptSolid", Items=[opening_solid])
                opening.Representation = model.create_entity("IfcProductDefinitionShape", Representations=[opening_rep])
                opening.ObjectPlacement = model.create_entity("IfcLocalPlacement", PlacementRelTo=best_wall["entity"].ObjectPlacement, RelativePlacement=model.create_entity("IfcAxis2Placement3D", Location=model.create_entity("IfcCartesianPoint", Coordinates=(0.,0.,0.))))
                
                # Apply void to wall
                try:
                    ifcopenshell.api.run("void.add_void", model, element=best_wall["entity"], opening=opening)
                except:
                    try:
                        ifcopenshell.api.run("feature.add_feature", model, feature=opening, element=best_wall["entity"])
                    except Exception as ex:
                        print(f"[WARN] Could not add opening void/feature: {ex}")

                # 2. Create Window/Door Element geometry (filling)
                filling = ifcopenshell.api.run("root.create_entity", model, ifc_class=ifc_class, name=f"{name_attr}_{e['id']}")
                ifcopenshell.api.run("spatial.assign_container", model, products=[filling], relating_structure=storey)
                
                # Create a brand new profile/solid so it doesn't share reference with opening void
                filling_profile = self.create_profile(model, item_poly)
                filling_solid = self.create_extrusion(model, filling_profile, z_elev, z_height)
                
                filling_rep = model.create_entity("IfcShapeRepresentation", ContextOfItems=context, RepresentationIdentifier="Body", RepresentationType="SweptSolid", Items=[filling_solid])
                filling.Representation = model.create_entity("IfcProductDefinitionShape", Representations=[filling_rep])
                filling.ObjectPlacement = model.create_entity("IfcLocalPlacement", PlacementRelTo=opening.ObjectPlacement, RelativePlacement=model.create_entity("IfcAxis2Placement3D", Location=model.create_entity("IfcCartesianPoint", Coordinates=(0.,0.,0.))))
                
                # Apply filling to opening
                try:
                    ifcopenshell.api.run("geometry.add_filling", model, opening=opening, element=filling)
                except:
                    try:
                        ifcopenshell.api.run("feature.add_filling", model, opening=opening, element=filling)
                    except Exception as ex:
                        print(f"[WARN] Could not add filling: {ex}")
            else:
                pass

        # 4. PROCESS ROOMS USING MAPPING
        for e in data.get('rooms', []):
            label_raw = e.get('label', '')
            label = self.normalize_label(label_raw)
            map_info = self.mapping.get(label)
            
            if not map_info:
                print(f"[WARN] Mapping not found for room: '{label_raw}'")
                continue
                
            ifc_class = map_info.get('IfcEntity')
            print(f"[INFO] Processing room: {label_raw} | IFC class: {ifc_class}")
            
            name_attr = map_info.get('Name', label_raw)
            poly = Polygon(self.get_coords(e['vertices'], canvas_h))
            element = ifcopenshell.api.run("root.create_entity", model, ifc_class=ifc_class, name=name_attr)
            
            predef_type = map_info.get('PredefinedType')
            if predef_type:
                try: element.PredefinedType = predef_type
                except: pass
                    
            obj_type = map_info.get('ObjectType')
            if obj_type:
                try: element.ObjectType = obj_type
                except: pass
                
            psets = map_info.get('Psets', {})
            if psets:
                for pset_name, props in psets.items():
                    try:
                        pset = ifcopenshell.api.run("pset.add_pset", model, product=element, name=pset_name)
                        ifcopenshell.api.run("pset.edit_pset", model, pset=pset, properties=props)
                    except Exception as ex:
                        print(f"[WARN] Failed to add Pset {pset_name}: {ex}")
            
            if ifc_class == "IfcSpace":
                ifcopenshell.api.run("aggregate.assign_object", model, products=[element], relating_object=storey)
            else:
                ifcopenshell.api.run("spatial.assign_container", model, products=[element], relating_structure=storey)
            
            profile = self.create_profile(model, poly)
            solid = self.create_extrusion(model, profile, z_elevation=0.0, z_height=self.height)
            
            shape_rep = model.create_entity("IfcShapeRepresentation", ContextOfItems=context, RepresentationIdentifier="Body", RepresentationType="SweptSolid", Items=[solid])
            element.Representation = model.create_entity("IfcProductDefinitionShape", Representations=[shape_rep])
            element.ObjectPlacement = model.create_entity("IfcLocalPlacement", PlacementRelTo=storey.ObjectPlacement, RelativePlacement=model.create_entity("IfcAxis2Placement3D", Location=model.create_entity("IfcCartesianPoint", Coordinates=(0.,0.,0.))))

        model.write(output_path)
        print(f"File successfully generated at: {output_path}")

if __name__ == "__main__":
    pipeline = BIMPipeline(scale=50.0, default_wall_height=3000.0)
    pipeline.run("floorplan_polygons (1).json", "hackathon_result.ifc")